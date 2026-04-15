#!/usr/bin/env python3
"""
SafeController — Capa de control seguro para el brazo robótico.

Todo movimiento de servo DEBE pasar por este módulo.
Es la ÚNICA puerta de acceso permitida al hardware de articulaciones.

Modelo de concurrencia
----------------------
Se usan tres primitivas de sincronización:

  _state_lock   threading.Lock   Protege _angles, _emergency, _last_cmd_t, _position_known.
                                 NUNCA se duerme mientras está tomado.

  HW_LOCK       threading.Lock   Definido en arm_system.hw_bus.
                                 Compartido con ControladorServo (modo autónomo).
                                 Se adquiere por cada paso de interpolación y se libera
                                 inmediatamente después. El sleep ocurre FUERA de este lock.

  _stop_event   threading.Event  Al activarse, el loop de interpolación en curso lo detecta
                                 en su próxima iteración y aborta. Tiempo de reacción: STEP_DELAY.

Flujo de move_safe()
--------------------
  Fase 1 — Validación (dentro de _state_lock, sin dormir):
    1. ¿Emergency activo?       → rechazar
    2. ¿Joint válido?           → rechazar
    3. ¿Rate limit ok?          → rechazar si < MIN_INTERVAL_S desde último comando
    4. Clamp al rango seguro
    5. ¿Salto > MAX_JUMP[joint]? → rechazar
    6. Calcular lista de pasos de interpolación
    7. Reservar _last_cmd_t = ahora
  (liberar _state_lock)

  Fase 2 — Interpolación (SIN locks, con accesos puntuales):
    Para cada step_angle:
      a. ¿_stop_event activo?         → abortar (emergency interrumpió)
      b. HW_LOCK.acquire(timeout=0.2) → si falla, rechazar (hardware ocupado)
      c. arm.set_joint_angle(joint, step_angle, smooth=False)  ← un pulso atómico
      d. HW_LOCK.release()
      e. _state_lock.acquire()
         _angles[joint] = step_angle
         _state_lock.release()
      f. time.sleep(STEP_DELAY)       ← FUERA de ambos locks

  Fase 3 — Confirmación:
    _state_lock.acquire()
    _angles[joint] = target (corrección final)
    _state_lock.release()

Uso básico:
    from arm_system.safety.safe_controller import SafeController

    with SafeController() as ctrl:
        ctrl.move_safe('shoulder', 90.0)
        ctrl.move_relative('elbow', -10.0)
        ctrl.go_home()
"""

from __future__ import annotations

import json
import logging
import threading
import time
from pathlib import Path
from typing import Dict, Iterable, List, Optional

log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Constantes de seguridad
# ---------------------------------------------------------------------------

# Tiempo entre pasos de interpolación (velocidad del servo: ~3°/20ms ≈ 150°/s)
STEP_DEG: float = 3.0
STEP_DELAY: float = 0.020

# Rate limit: mínimo tiempo entre comandos externos consecutivos
MIN_INTERVAL_S: float = 0.08

# Límite de salto por articulación (shoulder/elbow más restrictivos por su carga)
_MAX_JUMP_DEG: Dict[str, float] = {
    "shoulder": 20.0,
    "elbow":    25.0,
    "base":     30.0,
    "wrist":    40.0,
    "gripper":  60.0,
}
_MAX_JUMP_DEFAULT: float = 30.0

# Timeout para adquirir HW_LOCK en cada paso de interpolación
_HW_LOCK_TIMEOUT: float = 0.20

# Orden seguro para ir a home (descarga peso antes de mover shoulder/base)
_HOME_ORDER = ("gripper", "wrist", "elbow", "shoulder", "base")

# ---------------------------------------------------------------------------
_DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "servo_config.json"


class SafeController:
    """
    Capa de seguridad centralizada para control de servos.

    Envuelve ArmController añadiendo todas las validaciones antes de enviar
    cualquier pulso al hardware. En modo simulación registra los movimientos
    en log sin acceder al PCA9685.

    Protecciones implementadas:
      A. Limitación de ángulo    — clamp a [angle_safe_min, angle_safe_max] del JSON
      B. Rate limiting           — rechaza si < MIN_INTERVAL_S desde el último comando
      C. Suavizado               — interpolación en pasos de STEP_DEG dentro de SafeController
      D. Rechazo de salto brusco — rechaza si delta > MAX_JUMP por articulación
      E. Manejo de excepciones   — cualquier error de hardware activa emergency_stop
      F. Emergency stop          — corta PWM, bloquea futuros comandos, señaliza _stop_event
    """

    def __init__(
        self,
        config_path: Optional[Path] = None,
        *,
        simulation_mode: bool = False,
    ) -> None:
        self._state_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._emergency: bool = False
        self._last_cmd_t: float = 0.0
        self._position_known: bool = False
        self._arm = None
        self._sim: bool = simulation_mode
        self._angles: Dict[str, float] = {}
        self._limits: Dict[str, Dict[str, float]] = {}

        cfg = Path(config_path) if config_path else _DEFAULT_CONFIG
        self._limits = self._load_limits(cfg)

        if not self._sim:
            self._sim = not self._init_hardware(cfg)

        # Poblar ángulos lógicos desde el controlador o desde los defaults del JSON.
        # ADVERTENCIA: estos valores reflejan angle_home_deg, no la posición física real.
        # Llama a go_home() para sincronizar hardware y estado lógico.
        if self._arm is not None:
            for k in self._arm.iter_joint_keys():
                self._angles[k] = self._arm.get_joint_angle(k)
        else:
            for k, v in self._limits.items():
                self._angles[k] = v.get("home", 90.0)

        if self._sim:
            # En simulación asumimos posición home como punto de partida válido.
            self._position_known = True

        mode_tag = "SIMULACIÓN" if self._sim else "HARDWARE"
        log.info(
            "[SafeCtrl] Inicializado en modo %s. Joints: %s",
            mode_tag,
            sorted(self._angles.keys()),
        )
        if not self._sim and not self._position_known:
            log.warning(
                "[SafeCtrl] Posición física desconocida. "
                "Llama go_home() antes de usar move_relative()."
            )

    # -----------------------------------------------------------------------
    # Inicialización interna
    # -----------------------------------------------------------------------

    def _init_hardware(self, config_path: Path) -> bool:
        """Intenta conectar con el ArmController real. Retorna True si OK."""
        try:
            from arm_system.control.arm_controller import ArmController
        except ImportError:
            try:
                from control.arm_controller import ArmController  # type: ignore
            except ImportError as exc:
                log.error(
                    "[SafeCtrl] No se puede importar ArmController: %s. "
                    "Activando SIMULACIÓN.",
                    exc,
                )
                return False

        try:
            self._arm = ArmController(config_path=config_path)
            log.info("[SafeCtrl] ArmController conectado OK (I2C/PCA9685).")
            return True
        except Exception as exc:
            log.error(
                "[SafeCtrl] No se pudo conectar con hardware: %s. "
                "Activando SIMULACIÓN automáticamente.",
                exc,
            )
            return False

    def _load_limits(self, config_path: Path) -> Dict[str, Dict[str, float]]:
        """Carga límites de ángulo seguros desde servo_config.json."""
        defaults: Dict[str, Dict[str, float]] = {
            "base":     {"min": 0.0,  "max": 180.0, "home": 90.0},
            "shoulder": {"min": 15.0, "max": 165.0, "home": 90.0},
            "elbow":    {"min": 20.0, "max": 160.0, "home": 90.0},
            "wrist":    {"min": 20.0, "max": 160.0, "home": 90.0},
            "gripper":  {"min": 25.0, "max": 120.0, "home": 90.0},
        }
        try:
            with config_path.open("r", encoding="utf-8") as f:
                data = json.load(f)
            joints = data.get("joints") or {}
            result: Dict[str, Dict[str, float]] = {}
            for key, raw in joints.items():
                if key.startswith("_"):
                    continue
                result[key] = {
                    "min":  float(raw.get("angle_safe_min_deg", 0.0)),
                    "max":  float(raw.get("angle_safe_max_deg", 180.0)),
                    "home": float(raw.get("angle_home_deg", 90.0)),
                }
            if result:
                log.info("[SafeCtrl] Límites cargados desde %s", config_path)
                return result
        except Exception as exc:
            log.warning(
                "[SafeCtrl] No se pudo leer %s (%s). Usando límites por defecto.",
                config_path,
                exc,
            )
        return defaults

    # -----------------------------------------------------------------------
    # API pública — movimiento
    # -----------------------------------------------------------------------

    def move_safe(self, joint: str, angle: float) -> bool:
        """
        Mueve la articulación al ángulo absoluto indicado aplicando todas las
        protecciones de seguridad (A–F).

        El suavizado ocurre siempre (paso máximo STEP_DEG, pausa STEP_DELAY entre pasos).
        La pausa ocurre FUERA de cualquier lock para no bloquear el emergency stop.

        Args:
            joint:  Nombre de la articulación (base / shoulder / elbow / wrist / gripper).
            angle:  Ángulo objetivo en grados.

        Returns:
            True  → movimiento ejecutado correctamente.
            False → rechazado (seguridad, fallo de hardware, hardware ocupado).
        """
        # ── FASE 1: Validación dentro del state_lock (sin dormir) ────────────
        with self._state_lock:
            if self._emergency:
                log.error(
                    "[SafeCtrl] BLOQUEADO — emergency stop activo. "
                    "Llama reset_emergency() cuando sea seguro reanudar."
                )
                return False

            if joint not in self._angles:
                log.warning(
                    "[SafeCtrl] Articulación desconocida: '%s'. Válidas: %s",
                    joint,
                    sorted(self._angles.keys()),
                )
                return False

            # B: Rate limiting — rechazar, no dormir
            elapsed = time.monotonic() - self._last_cmd_t
            if elapsed < MIN_INTERVAL_S:
                log.warning(
                    "[SafeCtrl] Rate limit: comando demasiado rápido "
                    "(%.0f ms desde el último). Rechazado.",
                    elapsed * 1000,
                )
                return False

            # A: Clamp de ángulo
            clamped = self._clamp(joint, float(angle))

            # Optimización: si ya está en el destino, no hacer nada
            current = self._angles[joint]
            delta = abs(clamped - current)
            if delta < 0.5:
                return True

            # D: Rechazo de salto brusco
            max_jump = _MAX_JUMP_DEG.get(joint, _MAX_JUMP_DEFAULT)
            if delta > max_jump:
                log.warning(
                    "[SafeCtrl] RECHAZADO — salto peligroso en '%s': %.1f° "
                    "(actual=%.1f° → solicitado=%.1f°, máx=%.1f°)",
                    joint, delta, current, clamped, max_jump,
                )
                return False

            # Calcular pasos de interpolación
            steps = self._build_steps(current, clamped)

            # Reservar el slot de tiempo ANTES de comenzar la interpolación
            self._last_cmd_t = time.monotonic()

        # ── FASE 2: Interpolación (locks liberados; sleep fuera de ambos) ────
        if self._sim:
            return self._interpolate_sim(joint, current, clamped, steps)
        else:
            return self._interpolate_hw(joint, clamped, steps)

    def move_relative(self, joint: str, delta_deg: float) -> bool:
        """
        Mueve una articulación sumando delta_deg al ángulo actual.
        Requiere que _position_known sea True (llama a go_home() primero).

        Ejemplo:
            ctrl.move_relative('shoulder', 10.0)   # 10° hacia arriba
            ctrl.move_relative('shoulder', -10.0)  # 10° hacia abajo
        """
        with self._state_lock:
            if not self._position_known:
                log.error(
                    "[SafeCtrl] move_relative rechazado: posición desconocida. "
                    "Llama go_home() primero para sincronizar."
                )
                return False
            current = self._angles.get(joint, 90.0)
            target = current + delta_deg
        # La lectura de current ocurre dentro del lock.
        # move_safe adquirirá el lock de nuevo para sus propias validaciones.
        return self.move_safe(joint, target)

    # -----------------------------------------------------------------------
    # API pública — macros de alto nivel
    # -----------------------------------------------------------------------

    def go_home(self) -> bool:
        """
        Lleva todas las articulaciones a la posición home definida en
        servo_config.json, en orden seguro: gripper → wrist → elbow → shoulder → base.

        Usa move_safe() para cada joint, por lo que aplican TODAS las protecciones.
        Tras completar con éxito, marca _position_known = True.
        """
        with self._state_lock:
            if self._emergency:
                log.error("[SafeCtrl] BLOQUEADO — emergency stop activo.")
                return False

        joints_in_order = [j for j in _HOME_ORDER if j in self._limits]
        # Añadir joints que no estén en el orden predefinido al final
        joints_in_order += [j for j in self._limits if j not in joints_in_order]

        log.info("[SafeCtrl] Iniciando secuencia HOME: %s", joints_in_order)

        for joint in joints_in_order:
            home_angle = self._limits[joint].get("home", 90.0)
            ok = self.move_safe(joint, home_angle)
            if not ok:
                log.error(
                    "[SafeCtrl] go_home() falló en joint '%s'. "
                    "Abortando secuencia HOME.",
                    joint,
                )
                return False

        with self._state_lock:
            self._position_known = True

        log.info("[SafeCtrl] Posición HOME alcanzada. Estado lógico sincronizado.")
        return True

    def open_gripper(self) -> bool:
        """Abre la pinza al ángulo mínimo seguro (angle_safe_min_deg)."""
        open_angle = self._limits.get("gripper", {}).get("min", 25.0)
        return self.move_safe("gripper", open_angle)

    def close_gripper(self) -> bool:
        """Cierra la pinza al ángulo máximo seguro (angle_safe_max_deg)."""
        close_angle = self._limits.get("gripper", {}).get("max", 115.0)
        return self.move_safe("gripper", close_angle)

    # -----------------------------------------------------------------------
    # API pública — emergency stop
    # -----------------------------------------------------------------------

    def emergency_stop(self) -> None:
        """
        Para TODOS los servos cortando la señal PWM y bloquea todos los
        futuros comandos de movimiento hasta que se llame reset_emergency().

        Seguro para llamar desde cualquier thread y cualquier contexto.
        """
        self._do_emergency("llamada manual a emergency_stop()")

    def reset_emergency(self) -> None:
        """
        Reinicia el flag de emergencia permitiendo reanudar el control.

        ADVERTENCIA: Tras el reset, _position_known se pone a False.
        Se REQUIERE llamar a go_home() antes de usar move_relative().
        Solo llamar cuando el brazo esté en una posición segura y un
        operador haya verificado físicamente que es seguro reanudar.
        """
        with self._state_lock:
            if not self._emergency:
                return
            self._emergency = False
            self._position_known = False

        self._stop_event.clear()

        log.warning(
            "[SafeCtrl] Emergency stop REINICIADO. "
            "Llama go_home() antes de usar move_relative()."
        )

    # -----------------------------------------------------------------------
    # Propiedades e información de estado
    # -----------------------------------------------------------------------

    @property
    def is_emergency(self) -> bool:
        """True si el emergency stop está activo."""
        return self._emergency

    @property
    def is_simulation(self) -> bool:
        """True si está operando en modo simulación (sin hardware real)."""
        return self._sim

    @property
    def position_known(self) -> bool:
        """True si la posición ha sido sincronizada con el hardware (go_home completado)."""
        return self._position_known

    @property
    def joints(self):
        """Especificaciones de joints del ArmController. Dict vacío en simulación."""
        if self._arm is not None:
            return self._arm.joints
        return {}

    def get_angle(self, joint: str) -> float:
        """Retorna el último ángulo lógico conocido del joint."""
        with self._state_lock:
            return float(self._angles.get(joint, 0.0))

    def get_all_angles(self) -> Dict[str, float]:
        """Retorna una copia de todos los ángulos lógicos actuales."""
        with self._state_lock:
            return dict(self._angles)

    def iter_joint_keys(self) -> Iterable[str]:
        """Iterador sobre los nombres de articulación en orden estable."""
        return iter(sorted(self._angles.keys()))

    # -----------------------------------------------------------------------
    # Lifecycle
    # -----------------------------------------------------------------------

    def close(self) -> None:
        """
        Lleva el brazo a reposo y libera el hardware I2C de forma segura.
        Si hay emergency activo, solo libera PWM sin mover servos.
        """
        if self._arm is not None and not self._sim:
            try:
                if not self._emergency:
                    self._arm.close()
                else:
                    self._arm.release_all_pwm()
                log.info("[SafeCtrl] Hardware liberado correctamente.")
            except Exception as exc:
                log.warning("[SafeCtrl] Error al liberar hardware: %s", exc)

    def __enter__(self) -> "SafeController":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    # -----------------------------------------------------------------------
    # Internos — lógica de interpolación
    # -----------------------------------------------------------------------

    @staticmethod
    def _build_steps(current: float, target: float) -> List[float]:
        """
        Genera la lista de ángulos intermedios para la interpolación.
        El último elemento es siempre el target exacto.
        """
        delta = target - current
        if abs(delta) < 1e-6:
            return [target]
        direction = 1.0 if delta > 0 else -1.0
        n_steps = int(abs(delta) / STEP_DEG)
        steps: List[float] = []
        pos = current
        for _ in range(n_steps):
            pos += direction * STEP_DEG
            steps.append(pos)
        if not steps or abs(steps[-1] - target) > 1e-6:
            steps.append(target)
        return steps

    def _interpolate_hw(self, joint: str, target: float, steps: List[float]) -> bool:
        """
        Ejecuta el loop de interpolación sobre hardware real.
        Cada paso adquiere HW_LOCK de forma breve; el sleep ocurre fuera de ambos locks.

        Patrón de lock: adquirir → intentar escritura → liberar SIEMPRE en finally →
        LUEGO (fuera del lock) gestionar error. Así _do_emergency() puede adquirir
        HW_LOCK sin deadlock cuando lo llama la misma cadena de ejecución.
        """
        from arm_system import hw_bus  # importación local para evitar ciclos en tests

        for step_angle in steps:
            # ── Comprobar señal de parada (emergency desde otro thread) ──────
            if self._stop_event.is_set():
                log.warning(
                    "[SafeCtrl] Interpolación de '%s' interrumpida por emergency stop.",
                    joint,
                )
                return False

            # ── Adquirir HW_LOCK para el pulso atómico ──────────────────────
            if not hw_bus.HW_LOCK.acquire(timeout=_HW_LOCK_TIMEOUT):
                log.warning(
                    "[SafeCtrl] HW_LOCK no disponible en %.0f ms "
                    "(hardware ocupado por modo autónomo). Comando rechazado.",
                    _HW_LOCK_TIMEOUT * 1000,
                )
                return False

            # Capturar excepción sin HW_LOCK en el handler: liberar en finally,
            # gestionar el error DESPUÉS de que el finally haya ejecutado.
            _hw_error: Optional[Exception] = None
            try:
                self._arm.set_joint_angle(joint, step_angle, smooth=False)
            except Exception as exc:
                _hw_error = exc
            finally:
                hw_bus.HW_LOCK.release()  # liberar exactamente una vez, siempre

            # ── Gestionar error (ya sin HW_LOCK tomado) ──────────────────────
            if _hw_error is not None:
                log.error(
                    "[SafeCtrl] Error de hardware al mover '%s' a %.1f°: %s",
                    joint, step_angle, _hw_error,
                )
                self._do_emergency(
                    f"excepción en interpolación de '{joint}' → {step_angle:.1f}°: {_hw_error}"
                )
                return False

            # ── Actualizar estado lógico ─────────────────────────────────────
            with self._state_lock:
                self._angles[joint] = step_angle

            # ── Pausa FUERA de ambos locks ───────────────────────────────────
            time.sleep(STEP_DELAY)

        # Corrección final del ángulo lógico
        with self._state_lock:
            self._angles[joint] = target

        log.info("[SafeCtrl] %s: %.1f° completado.", joint, target)
        return True

    def _interpolate_sim(
        self, joint: str, start: float, target: float, steps: List[float]
    ) -> bool:
        """
        Ejecuta la interpolación en modo simulación.
        Respeta STEP_DELAY para hacer el timing realista (MG996R ≈ 150°/s).
        """
        for step_angle in steps:
            if self._stop_event.is_set():
                log.warning("[SafeCtrl] [SIM] Interpolación de '%s' interrumpida.", joint)
                return False
            with self._state_lock:
                self._angles[joint] = step_angle
            time.sleep(STEP_DELAY)  # Simula velocidad real del servo

        with self._state_lock:
            self._angles[joint] = target

        log.info("[SafeCtrl] [SIM] %s: %.1f° → %.1f°", joint, start, target)
        return True

    # -----------------------------------------------------------------------
    # Internos — clamp y emergency
    # -----------------------------------------------------------------------

    def _clamp(self, joint: str, angle: float) -> float:
        """Limita el ángulo al rango seguro del joint. Debe llamarse dentro de _state_lock."""
        limits = self._limits.get(joint, {"min": 0.0, "max": 180.0})
        lo = min(float(limits["min"]), float(limits["max"]))
        hi = max(float(limits["min"]), float(limits["max"]))
        clamped = max(lo, min(hi, angle))
        if abs(clamped - angle) > 0.5:
            log.warning(
                "[SafeCtrl] Ángulo ajustado: %s %.1f° → %.1f° "
                "(rango seguro [%.0f°, %.0f°])",
                joint, angle, clamped, lo, hi,
            )
        return clamped

    def _do_emergency(self, reason: str) -> None:
        """
        Activa el emergency stop de forma segura desde cualquier contexto.

        Orden de operaciones:
          1. Señalizar _stop_event → el loop de interpolación aborta en su próxima iteración
          2. Setear _emergency dentro de _state_lock → bloquea nuevos comandos
          3. Intentar release_all_pwm() via HW_LOCK → cortar señal al PCA9685

        No requiere que el caller tenga ningún lock tomado.
        """
        # Paso 1: señalizar parada inmediata al loop de interpolación
        self._stop_event.set()

        # Paso 2: marcar estado de emergencia
        with self._state_lock:
            self._emergency = True

        log.critical("[SafeCtrl] *** EMERGENCY STOP *** Motivo: %s", reason)

        # Paso 3: cortar PWM — intentar con HW_LOCK; si no está disponible, forzar igualmente
        if self._arm is not None and not self._sim:
            from arm_system import hw_bus

            acquired = hw_bus.HW_LOCK.acquire(timeout=0.5)
            try:
                self._arm.release_all_pwm()
                log.critical("[SafeCtrl] PWM cortado en todos los canales del PCA9685.")
            except Exception as exc:
                log.critical(
                    "[SafeCtrl] No se pudo cortar PWM (hardware inaccesible): %s", exc
                )
            finally:
                if acquired:
                    hw_bus.HW_LOCK.release()
