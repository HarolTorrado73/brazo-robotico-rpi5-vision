"""
Controlador de brazo robótico estático vía PCA9685 (I2C).

Solo servomotores: sin motores DC, ultrasonidos, sensores IR de línea ni LED RGB.
La conversión ángulo → ancho de pulso sigue el enfoque habitual en proyectos tipo
Adeept (PCA9685 a 50 Hz), con arranque a "home" por pasos discretos (más suave que
un salto brusco de PWM, dentro de lo que permite un servo sin realimentación de posición).
"""

from __future__ import annotations

import json
import logging
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Union

import board
import busio
from adafruit_pca9685 import PCA9685

log = logging.getLogger(__name__)

Number = Union[int, float]

DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent.parent / "servo_config.json"


@dataclass(frozen=True)
class JointSpec:
    """Especificación de un grado de libertad (servo en un canal del PCA9685)."""

    key: str
    channel: int
    description: str
    angle_safe_min_deg: float
    angle_safe_max_deg: float
    angle_home_deg: float
    pulse_min_us: int
    pulse_max_us: int
    invert: bool
    angle_open_deg: Optional[float] = None
    angle_close_deg: Optional[float] = None

    def clamp_angle(self, angle_deg: Number) -> float:
        """Limita el ángulo al rango seguro declarado en configuración."""
        lo = float(self.angle_safe_min_deg)
        hi = float(self.angle_safe_max_deg)
        if lo > hi:
            lo, hi = hi, lo
        v = float(angle_deg)
        return max(lo, min(hi, v))


class ArmController:
    """
    Orquesta los cuatro servos del brazo (base, hombro, codo, pinza) usando un JSON
    de calibración y el bus I2C de la Raspberry Pi (p. ej. GPIO2/GPIO3 en RPi 5).
    """

    JOINT_BASE = "base"
    JOINT_SHOULDER = "shoulder"
    JOINT_ELBOW = "elbow"
    JOINT_GRIPPER = "gripper"
    KNOWN_JOINTS = (JOINT_BASE, JOINT_SHOULDER, JOINT_ELBOW, JOINT_GRIPPER)

    def __init__(
        self,
        config_path: Optional[Path] = None,
        *,
        i2c_address: Optional[int] = None,
    ) -> None:
        """
        Inicializa el bus I2C y el PCA9685. No mueve los servos hasta que llames a
        :meth:`initialize_to_home_smooth` (o a otro método de movimiento).

        Args:
            config_path: Ruta a ``servo_config.json``. Por defecto, junto a este módulo.
            i2c_address: Dirección I2C del PCA9685; si es None, se toma del JSON.
        """
        self._config_path = Path(config_path) if config_path else DEFAULT_CONFIG_PATH
        raw = self._read_json(self._config_path)
        self._pca_cfg: Dict[str, Any] = dict(raw.get("pca9685") or {})
        self._motion_cfg: Dict[str, Any] = dict(raw.get("motion") or {})
        self._home_sequence: List[str] = list(raw.get("home_sequence") or list(self.KNOWN_JOINTS))

        addr_hex = self._pca_cfg.get("i2c_address_hex", "0x40")
        freq = int(self._pca_cfg.get("pwm_frequency_hz", 50))
        if i2c_address is not None:
            self._i2c_address = int(i2c_address)
        else:
            self._i2c_address = int(str(addr_hex), 16)

        self._joints: Dict[str, JointSpec] = self._parse_joints(raw.get("joints") or {})

        self._rest_margin_deg = float(self._motion_cfg.get("rest_margin_deg", 4.0))
        self._rest_settle_s = float(self._motion_cfg.get("rest_settle_s", 0.25))
        _rs = self._motion_cfg.get("rest_sequence")
        self._rest_sequence: List[str] = (
            list(_rs) if isinstance(_rs, list) and _rs else ["gripper", "elbow", "shoulder", "base"]
        )

        try:
            self.i2c = busio.I2C(board.D3, board.D2)
            self._pca = PCA9685(self.i2c, address=self._i2c_address)
            self._pca.frequency = freq
        except Exception as exc:
            msg = (
                "Error: No se detecta el PCA9685. Verifica el cableado I2C y la alimentación "
                "(SDA→GPIO2 pin 3, SCL→GPIO3 pin 5; I2C activo en raspi-config; dirección típica 0x40; "
                "VCC/GND del driver y masa común con la Raspberry Pi)."
            )
            log.error("%s Detalle técnico: %s", msg, exc)
            print(msg, file=sys.stderr)
            sys.exit(1)

        log.info(
            "PCA9685 listo: dirección 0x%02X, %d Hz, config=%s",
            self._i2c_address,
            freq,
            self._config_path,
        )

        # Modelo lógico de posición (sin encoder; sincroniza con la realidad al calibrar o asumir).
        self._angles_deg: Dict[str, float] = {
            k: float(v.angle_home_deg) for k, v in self._joints.items()
        }

    # --- Ciclo de vida hardware -------------------------------------------------
    def close(self) -> None:
        """
        Lleva el brazo a postura de reposo (carga apoyada / articulaciones bajas), luego corta PWM
        y libera el bus I2C. Si falla el reposo, intenta igualmente apagar salidas.
        """
        try:
            try:
                self.go_to_rest_position()
            except Exception as exc:
                log.warning("go_to_rest_position falló antes de cortar PWM: %s", exc)
            self.release_all_pwm()
        finally:
            try:
                self._pca.deinit()
            except Exception as exc:  # pragma: no cover
                log.debug("PCA9685 deinit: %s", exc)
            try:
                self.i2c.deinit()
            except Exception as exc:  # pragma: no cover
                log.debug("I2C deinit: %s", exc)

    def __enter__(self) -> "ArmController":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    # --- Configuración -----------------------------------------------------------
    @staticmethod
    def _read_json(path: Path) -> Dict[str, Any]:
        if not path.is_file():
            raise FileNotFoundError(f"No existe el archivo de configuración: {path}")
        with path.open("r", encoding="utf-8") as f:
            return json.load(f)

    def _parse_joints(self, data: Mapping[str, Any]) -> Dict[str, JointSpec]:
        out: Dict[str, JointSpec] = {}
        for key, raw in data.items():
            if key.startswith("_"):
                continue
            if key not in self.KNOWN_JOINTS:
                log.warning("Articulación '%s' ignorada (solo se admiten: %s)", key, self.KNOWN_JOINTS)
                continue
            try:
                out[key] = JointSpec(
                    key=key,
                    channel=int(raw["channel"]),
                    description=str(raw.get("description", "")),
                    angle_safe_min_deg=float(raw["angle_safe_min_deg"]),
                    angle_safe_max_deg=float(raw["angle_safe_max_deg"]),
                    angle_home_deg=float(raw["angle_home_deg"]),
                    pulse_min_us=int(raw["pulse_min_us"]),
                    pulse_max_us=int(raw["pulse_max_us"]),
                    invert=bool(raw.get("invert", False)),
                    angle_open_deg=(
                        float(raw["angle_open_deg"]) if raw.get("angle_open_deg") is not None else None
                    ),
                    angle_close_deg=(
                        float(raw["angle_close_deg"]) if raw.get("angle_close_deg") is not None else None
                    ),
                )
            except (KeyError, TypeError, ValueError) as e:
                raise ValueError(f"Joint '{key}' inválido en JSON: {e}") from e

        missing = [k for k in self.KNOWN_JOINTS if k not in out]
        if missing:
            raise ValueError(f"Faltan articulaciones obligatorias en JSON: {missing}")
        return out

    @property
    def joints(self) -> Mapping[str, JointSpec]:
        """Vista inmutable de las especificaciones cargadas."""
        return self._joints

    # --- PWM / geometría pulso ---------------------------------------------------
    @staticmethod
    def _pulse_to_duty(pulse_us: float) -> int:
        """50 Hz → período 20 ms; duty en escala 0..0xFFFF (CircuitPython PCA9685)."""
        return int(pulse_us / 20_000.0 * 0xFFFF)

    def _angle_to_pulse_us(self, spec: JointSpec, angle_deg: float) -> int:
        a = spec.clamp_angle(angle_deg)
        lo_a = float(spec.angle_safe_min_deg)
        hi_a = float(spec.angle_safe_max_deg)
        if lo_a > hi_a:
            lo_a, hi_a = hi_a, lo_a
        span_a = hi_a - lo_a
        if span_a <= 0:
            t = 0.0
        else:
            t = (a - lo_a) / span_a
        if spec.invert:
            t = 1.0 - t

        lo_p = int(spec.pulse_min_us)
        hi_p = int(spec.pulse_max_us)
        if lo_p > hi_p:
            lo_p, hi_p = hi_p, lo_p
        pulse = lo_p + t * (hi_p - lo_p)
        return int(round(max(lo_p, min(hi_p, pulse))))

    def _set_channel_pulse_us(self, channel: int, pulse_us: int) -> None:
        self._pca.channels[channel].duty_cycle = self._pulse_to_duty(pulse_us)

    def release_all_pwm(self) -> None:
        """Pone duty 0 en todos los canales usados (servos sin par; útil al apagar)."""
        for spec in self._joints.values():
            try:
                self._pca.channels[spec.channel].duty_cycle = 0
            except Exception as exc:  # pragma: no cover
                log.warning("No se pudo apagar canal %d: %s", spec.channel, exc)

    # --- Arranque tipo initPosServos (con pasos suaves) ---------------------------
    def initialize_to_home_smooth(
        self,
        assumed_positions_deg: Optional[Mapping[str, float]] = None,
    ) -> None:
        """
        Lleva todas las articulaciones a ``angle_home_deg`` recorriendo la secuencia
        definida en ``home_sequence`` del JSON.

        Inspiración: ``initPosServos.py`` de Adeept fija un pulso central; aquí se
        aproxima el mismo objetivo (posición inicial segura) interpolando en pasos de
        ángulo para reducir picos de corriente.

        Si no pasas ``assumed_positions_deg``, se asume que el modelo lógico ya está
        en home (sin recorrido intermedio) y solo se aplica el pulso final por pasos
        mínimos (un paso). Para un arranque más suave desde una postura conocida,
        pasa un diccionario p. ej. ``{"shoulder": 45.0, ...}``.
        """
        step = float(self._motion_cfg.get("home_step_deg", 2.0))
        delay = float(self._motion_cfg.get("home_step_delay_s", 0.025))
        assumed = dict(assumed_positions_deg) if assumed_positions_deg else {}

        for key in self._home_sequence:
            if key not in self._joints:
                log.warning("home_sequence: articulación desconocida '%s', omitida", key)
                continue
            spec = self._joints[key]
            start = float(assumed.get(key, self._angles_deg[key]))
            target = float(spec.angle_home_deg)
            self._smooth_transition_deg(key, start, target, step_deg=step, delay_s=delay)
            self._angles_deg[key] = spec.clamp_angle(target)

        log.info("Inicialización a HOME completada (secuencia=%s)", self._home_sequence)

    def go_to_rest_position(self) -> None:
        """
        Mueve suavemente hacia una postura de **reposo mecánico** (brazo más bajo / apoyado),
        antes de cortar la señal PWM: hombro y codo hacia el mínimo del rango seguro (con margen),
        pinza abierta, base en home. Ajusta ``invert`` y límites en el JSON si tu mecánica define
        “abajo” con el otro extremo del rango.

        La secuencia y márgenes se pueden tunear en ``servo_config.json`` → ``motion``:
        ``rest_sequence``, ``rest_margin_deg``, ``rest_settle_s``.
        """
        step = float(self._motion_cfg.get("default_move_step_deg", 3.0))
        delay = float(self._motion_cfg.get("default_move_delay_s", 0.02))
        m = self._rest_margin_deg

        for key in self._rest_sequence:
            if key not in self._joints:
                log.warning("rest_sequence: articulación desconocida '%s', omitida", key)
                continue
            spec = self._joints[key]
            if key == self.JOINT_GRIPPER:
                if spec.angle_open_deg is not None:
                    target = float(spec.clamp_angle(spec.angle_open_deg))
                else:
                    target = float(spec.clamp_angle(float(spec.angle_safe_min_deg) + m))
            elif key == self.JOINT_BASE:
                target = float(spec.angle_home_deg)
            else:
                target = float(spec.clamp_angle(float(spec.angle_safe_min_deg) + m))

            self._smooth_transition_deg(
                key, self._angles_deg[key], target, step_deg=step, delay_s=delay
            )
            self._angles_deg[key] = spec.clamp_angle(target)

        time.sleep(max(0.0, self._rest_settle_s))
        log.info("Postura de reposo alcanzada (secuencia=%s)", self._rest_sequence)

    def sync_logical_angles(self, positions_deg: Mapping[str, float]) -> None:
        """
        Alinea el estado lógico interno con una postura conocida (p. ej. tras montaje
        manual o visión). No envía PWM.
        """
        for key, ang in positions_deg.items():
            if key in self._joints:
                self._angles_deg[key] = self._joints[key].clamp_angle(ang)

    # --- Movimiento básico --------------------------------------------------------
    def set_joint_angle(
        self,
        joint: str,
        angle_deg: Number,
        *,
        smooth: bool = False,
        step_deg: Optional[float] = None,
        delay_s: Optional[float] = None,
    ) -> float:
        """
        Mueve un servo a un ángulo absoluto en grados, validando el rango seguro.

        Returns:
            Ángulo aplicado tras clamping.
        """
        if joint not in self._joints:
            raise KeyError(f"Articulación desconocida: {joint!r}")
        spec = self._joints[joint]
        target = spec.clamp_angle(angle_deg)
        if smooth:
            sd = float(step_deg if step_deg is not None else self._motion_cfg.get("default_move_step_deg", 3.0))
            dl = float(delay_s if delay_s is not None else self._motion_cfg.get("default_move_delay_s", 0.02))
            self._smooth_transition_deg(joint, self._angles_deg[joint], target, step_deg=sd, delay_s=dl)
        else:
            pulse = self._angle_to_pulse_us(spec, target)
            self._set_channel_pulse_us(spec.channel, pulse)
        self._angles_deg[joint] = target
        return target

    def _smooth_transition_deg(
        self,
        joint: str,
        start_deg: float,
        end_deg: float,
        *,
        step_deg: float,
        delay_s: float,
    ) -> None:
        spec = self._joints[joint]
        start = spec.clamp_angle(start_deg)
        end = spec.clamp_angle(end_deg)
        if step_deg <= 0:
            raise ValueError("step_deg debe ser > 0")
        delta = end - start
        if abs(delta) < 1e-6:
            pulse = self._angle_to_pulse_us(spec, end)
            self._set_channel_pulse_us(spec.channel, pulse)
            time.sleep(delay_s)
            return
        direction = 1.0 if delta > 0 else -1.0
        steps = int(abs(delta) / step_deg)
        remainder = abs(delta) - steps * step_deg
        current = start
        for _ in range(steps):
            current += direction * step_deg
            pulse = self._angle_to_pulse_us(spec, current)
            self._set_channel_pulse_us(spec.channel, pulse)
            time.sleep(delay_s)
        if remainder > 1e-6:
            current += direction * remainder
        pulse = self._angle_to_pulse_us(spec, end)
        self._set_channel_pulse_us(spec.channel, pulse)
        time.sleep(max(delay_s, 0.05))

    def get_joint_angle(self, joint: str) -> float:
        """Último ángulo lógico registrado para la articulación."""
        if joint not in self._joints:
            raise KeyError(f"Articulación desconocida: {joint!r}")
        return float(self._angles_deg[joint])

    # --- Macros legibles ----------------------------------------------------------
    def move_base(self, angle_deg: Number, *, smooth: bool = False) -> float:
        """Gira la base al ángulo indicado (izquierda/derecha según montaje mecánico)."""
        return self.set_joint_angle(self.JOINT_BASE, angle_deg, smooth=smooth)

    def move_shoulder(self, angle_deg: Number, *, smooth: bool = False) -> float:
        """Hombro arriba/abajo."""
        return self.set_joint_angle(self.JOINT_SHOULDER, angle_deg, smooth=smooth)

    def move_elbow(self, angle_deg: Number, *, smooth: bool = False) -> float:
        """Codo arriba/abajo."""
        return self.set_joint_angle(self.JOINT_ELBOW, angle_deg, smooth=smooth)

    def open_gripper(self, *, smooth: bool = True) -> float:
        """Abre la pinza usando ``angle_open_deg`` del JSON (o el mínimo seguro si falta)."""
        spec = self._joints[self.JOINT_GRIPPER]
        ang = spec.angle_open_deg if spec.angle_open_deg is not None else spec.angle_safe_min_deg
        return self.set_joint_angle(self.JOINT_GRIPPER, ang, smooth=smooth)

    def close_gripper(self, *, smooth: bool = True) -> float:
        """Cierra la pinza usando ``angle_close_deg`` del JSON (o el máximo seguro si falta)."""
        spec = self._joints[self.JOINT_GRIPPER]
        ang = spec.angle_close_deg if spec.angle_close_deg is not None else spec.angle_safe_max_deg
        return self.set_joint_angle(self.JOINT_GRIPPER, ang, smooth=smooth)

    # --- Extensión visión / OpenCV (pendiente) ----------------------------------
    def move_to_target(
        self,
        x: Number,
        y: Number,
        *,
        frame_width: Optional[int] = None,
        frame_height: Optional[int] = None,
        depth_z: Optional[float] = None,
    ) -> None:
        """
        Punto de enganche para un futuro pipeline de OpenCV (detección → cinemática).

        Aquí irá la cadena típica: coordenadas en imagen → (opcional) profundidad /
        estimación de pose → cinemática inversa → :meth:`set_joint_angle` en cada DOF.

        Raises:
            NotImplementedError: hasta integrar el modelo geométrico del brazo.
        """
        raise NotImplementedError(
            "move_to_target: pendiente de cinemática inversa / calibración cámara-brazo. "
            f"Recibido punto (x={x!r}, y={y!r}), frame=({frame_width!r}x{frame_height!r}), z={depth_z!r}"
        )

    def iter_joint_keys(self) -> Iterable[str]:
        """Nombres de articulación en orden estable."""
        return iter(self.KNOWN_JOINTS)
