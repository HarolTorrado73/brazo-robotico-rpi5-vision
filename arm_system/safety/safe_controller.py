#!/usr/bin/env python3
"""
SafeController — Capa de control seguro para el brazo robótico.

Todo movimiento de servo DEBE pasar por este módulo.
Es la ÚNICA puerta de acceso permitida al hardware de articulaciones.

Protecciones implementadas:
  A. Limitación de ángulo    — clamp a [angle_safe_min, angle_safe_max] del JSON
  B. Rate limiting           — mínimo 80 ms entre comandos
  C. Suavizado               — máximo 3° por paso de interpolación interna
  D. Rechazo de salto brusco — cambios > 40° de una vez son rechazados
  E. Manejo de excepciones   — cualquier error en hardware activa emergency_stop
  F. Emergency stop          — corta PWM, bloquea futuros comandos

Uso básico:
    from arm_system.safety.safe_controller import SafeController

    with SafeController() as ctrl:
        ctrl.move_safe('shoulder', 90.0)
        ctrl.move_relative('elbow', -10.0)
        ctrl.go_home()

Modo simulación (activo automáticamente si el hardware no está disponible):
    ctrl = SafeController(simulation_mode=True)
"""

from __future__ import annotations

import json
import logging
import threading
import time
from pathlib import Path
from typing import Dict, Iterable, Optional

log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Constantes de seguridad — ajustar con criterio, no sin razón
# ---------------------------------------------------------------------------
MAX_JUMP_DEG: float = 40.0    # Salto máximo permitido de una sola vez (°)
STEP_DEG: float = 3.0         # Paso máximo por ciclo de interpolación interna (°)
MIN_INTERVAL_S: float = 0.08  # Rate limit: mínimo tiempo entre comandos (80 ms)
# ---------------------------------------------------------------------------

_DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "servo_config.json"


class SafeController:
    """
    Capa de seguridad centralizada para control de servos.

    Envuelve :class:`ArmController` añadiendo todas las validaciones de
    seguridad antes de enviar cualquier pulso al hardware.  En modo
    simulación registra los movimientos en log sin acceder al PCA9685.
    """

    def __init__(
        self,
        config_path: Optional[Path] = None,
        *,
        simulation_mode: bool = False,
    ) -> None:
        self._lock = threading.Lock()
        self._emergency: bool = False
        self._last_cmd_t: float = 0.0
        self._arm = None
        self._sim: bool = simulation_mode
        self._angles: Dict[str, float] = {}
        self._limits: Dict[str, Dict[str, float]] = {}

        cfg = Path(config_path) if config_path else _DEFAULT_CONFIG
        self._limits = self._load_limits(cfg)

        if not self._sim:
            self._sim = not self._init_hardware(cfg)

        # Poblar ángulos conocidos desde el controlador o desde defaults
        if self._arm is not None:
            for k in self._arm.iter_joint_keys():
                self._angles[k] = self._arm.get_joint_angle(k)
        else:
            for k, v in self._limits.items():
                self._angles[k] = v.get("home", 90.0)

        mode_tag = "SIMULACIÓN" if self._sim else "HARDWARE"
        log.info(
            "[SafeCtrl] Inicializado en modo %s. Joints: %s",
            mode_tag,
            sorted(self._angles.keys()),
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
                # Ruta relativa cuando se importa desde dentro de arm_system
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

    def move_safe(self, joint: str, angle: float, *, smooth: bool = True) -> bool:
        """
        Mueve la articulación al ángulo indicado aplicando todas las
        protecciones de seguridad (A–F).

        Args:
            joint:  Nombre de la articulación (base / shoulder / elbow / wrist / gripper).
            angle:  Ángulo objetivo en grados.
            smooth: Si True, interpola en pasos de STEP_DEG (más suave).

        Returns:
            True  → movimiento ejecutado correctamente.
            False → rechazado por seguridad o fallo de hardware.
        """
        with self._lock:
            # ── F: Emergency stop activo ─────────────────────────────────
            if self._emergency:
                log.error(
                    "[SafeCtrl] BLOQUEADO — emergency stop activo. "
                    "Llama reset_emergency() cuando sea seguro reanudar."
                )
                return False

            # ── Validar joint ────────────────────────────────────────────
            if joint not in self._angles:
                log.warning(
                    "[SafeCtrl] Articulación desconocida: '%s'. Válidas: %s",
                    joint,
                    sorted(self._angles.keys()),
                )
                return False

            # ── B: Rate limiting ─────────────────────────────────────────
            self._apply_rate_limit()

            # ── A: Clamp de ángulo ───────────────────────────────────────
            clamped = self._clamp(joint, float(angle))

            # ── D: Rechazo de salto brusco ───────────────────────────────
            current = self._angles[joint]
            delta = abs(clamped - current)
            if delta > MAX_JUMP_DEG:
                log.warning(
                    "[SafeCtrl] RECHAZADO — salto peligroso en '%s': %.1f° "
                    "(actual=%.1f° → solicitado=%.1f°, máx=%.1f°)",
                    joint, delta, current, clamped, MAX_JUMP_DEG,
                )
                return False

            # ── C + E: Ejecutar con suavizado y manejo de errores ────────
            try:
                if self._sim:
                    self._angles[joint] = clamped
                    log.info(
                        "[SafeCtrl] [SIM] %s: %.1f° → %.1f°",
                        joint, current, clamped,
                    )
                else:
                    self._arm.set_joint_angle(
                        joint, clamped, smooth=smooth, step_deg=STEP_DEG
                    )
                    self._angles[joint] = self._arm.get_joint_angle(joint)
                    log.info(
                        "[SafeCtrl] %s: %.1f° → %.1f°",
                        joint, current, self._angles[joint],
                    )

                self._last_cmd_t = time.monotonic()
                return True

            except Exception as exc:
                log.error(
                    "[SafeCtrl] Error de hardware al mover '%s' a %.1f°: %s",
                    joint, clamped, exc,
                )
                self._do_emergency(
                    f"excepción en move_safe('{joint}', {clamped:.1f}°): {exc}"
                )
                return False

    def move_relative(self, joint: str, delta_deg: float) -> bool:
        """
        Mueve una articulación de forma relativa sumando delta_deg al ángulo
        actual.  Útil para controles de dirección (+1 / -1).

        Ejemplo:
            ctrl.move_relative('shoulder', 10.0)   # 10° hacia arriba
            ctrl.move_relative('shoulder', -10.0)  # 10° hacia abajo
        """
        current = self._angles.get(joint, 90.0)
        return self.move_safe(joint, current + delta_deg)

    # -----------------------------------------------------------------------
    # API pública — macros de alto nivel
    # -----------------------------------------------------------------------

    def go_home(self) -> bool:
        """
        Lleva todas las articulaciones a la posición home definida en
        servo_config.json de forma suave y segura.
        """
        if self._emergency:
            log.error("[SafeCtrl] BLOQUEADO — emergency stop activo.")
            return False

        if self._arm is not None and not self._sim:
            try:
                self._arm.initialize_to_home_smooth()
                for k in self._arm.iter_joint_keys():
                    self._angles[k] = self._arm.get_joint_angle(k)
                log.info("[SafeCtrl] Posición HOME alcanzada.")
                return True
            except Exception as exc:
                log.error("[SafeCtrl] Error al ir a HOME: %s", exc)
                self._do_emergency(f"error en go_home(): {exc}")
                return False
        else:
            # Simulación: mover cada joint a su home
            ok = True
            for k, v in self._limits.items():
                ok = self.move_safe(k, v.get("home", 90.0)) and ok
            return ok

    def open_gripper(self) -> bool:
        """Abre la pinza usando el ángulo definido en servo_config.json."""
        if self._arm is not None and not self._sim:
            try:
                ang = self._arm.open_gripper(smooth=True)
                self._angles["gripper"] = ang
                log.info("[SafeCtrl] Pinza ABIERTA → %.1f°", ang)
                return True
            except Exception as exc:
                log.error("[SafeCtrl] Error al abrir pinza: %s", exc)
                self._do_emergency(f"error en open_gripper(): {exc}")
                return False
        else:
            open_ang = self._limits.get("gripper", {}).get("min", 25.0)
            return self.move_safe("gripper", open_ang)

    def close_gripper(self) -> bool:
        """Cierra la pinza usando el ángulo definido en servo_config.json."""
        if self._arm is not None and not self._sim:
            try:
                ang = self._arm.close_gripper(smooth=True)
                self._angles["gripper"] = ang
                log.info("[SafeCtrl] Pinza CERRADA → %.1f°", ang)
                return True
            except Exception as exc:
                log.error("[SafeCtrl] Error al cerrar pinza: %s", exc)
                self._do_emergency(f"error en close_gripper(): {exc}")
                return False
        else:
            close_ang = self._limits.get("gripper", {}).get("max", 115.0)
            return self.move_safe("gripper", close_ang)

    # -----------------------------------------------------------------------
    # API pública — emergency stop
    # -----------------------------------------------------------------------

    def emergency_stop(self) -> None:
        """
        Para TODOS los servos cortando la señal PWM y bloquea todos los
        futuros comandos de movimiento hasta que se llame reset_emergency().
        """
        self._do_emergency("llamada manual a emergency_stop()")

    def reset_emergency(self) -> None:
        """
        Reinicia el flag de emergencia permitiendo reanudar el control.

        ADVERTENCIA: Solo llamar cuando el brazo esté en una posición segura
        y un operador haya verificado físicamente que es seguro reanudar.
        """
        if not self._emergency:
            return
        log.warning(
            "[SafeCtrl] Emergency stop REINICIADO por operador. "
            "Verificar posición del brazo antes de enviar comandos."
        )
        self._emergency = False

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
    def joints(self):
        """
        Expone las especificaciones de joints del ArmController para display.
        Retorna dict vacío en modo simulación (sin ArmController real).
        """
        if self._arm is not None:
            return self._arm.joints
        return {}

    def get_angle(self, joint: str) -> float:
        """Retorna el último ángulo lógico conocido del joint."""
        return float(self._angles.get(joint, 0.0))

    def get_all_angles(self) -> Dict[str, float]:
        """Retorna una copia de todos los ángulos lógicos actuales."""
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
    # Internos
    # -----------------------------------------------------------------------

    def _clamp(self, joint: str, angle: float) -> float:
        """Limita el ángulo al rango seguro del joint. Registra si ajusta."""
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

    def _apply_rate_limit(self) -> None:
        """Espera el tiempo necesario para respetar el rate limit de 80 ms."""
        elapsed = time.monotonic() - self._last_cmd_t
        if elapsed < MIN_INTERVAL_S:
            wait = MIN_INTERVAL_S - elapsed
            log.debug(
                "[SafeCtrl] Rate limit: esperando %.0f ms", wait * 1000
            )
            time.sleep(wait)

    def _do_emergency(self, reason: str) -> None:
        """Activa el emergency stop, corta PWM y bloquea futuros comandos."""
        self._emergency = True
        log.critical(
            "[SafeCtrl] *** EMERGENCY STOP *** Motivo: %s", reason
        )
        if self._arm is not None and not self._sim:
            try:
                self._arm.release_all_pwm()
                log.critical(
                    "[SafeCtrl] PWM cortado en todos los canales del PCA9685."
                )
            except Exception as exc:
                log.critical(
                    "[SafeCtrl] No se pudo cortar PWM (hardware inaccesible): %s",
                    exc,
                )
