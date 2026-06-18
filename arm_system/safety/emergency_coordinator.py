#!/usr/bin/env python3
"""Coordinador de emergencia para PR #1.

Este módulo centraliza el estado de emergencia y las acciones comunes de parada
sin refactorizar ControladorRobotico ni SafeController.
"""

from __future__ import annotations

import logging
import threading
from typing import Optional

log = logging.getLogger(__name__)

_SAFE_CONTROLLER = None
_CEREBRO = None
_STATE_LOCK = threading.Lock()
_EMERGENCY = False
_AUTONOMOUS_ACTIVE = False
_CUSTOM_SEQUENCE_ACTIVE = False


def get_emergency_coordinator():
    return EmergencyCoordinator


def _safe_msg(safe) -> str:
    try:
        if safe.is_emergency:
            return "emergency activo"
        if safe.is_simulation:
            return "simulación"
    except Exception:
        pass
    return "listo"


class EmergencyCoordinator:
    """Coordinador singleton para Emergency Stop y reset."""

    @staticmethod
    def attach_safe_controller(safe) -> None:
        global _SAFE_CONTROLLER
        with _STATE_LOCK:
            _SAFE_CONTROLLER = safe

    @staticmethod
    def attach_cerebro(cerebro) -> None:
        global _CEREBRO
        with _STATE_LOCK:
            _CEREBRO = cerebro

    @staticmethod
    def is_emergency() -> bool:
        with _STATE_LOCK:
            return bool(_EMERGENCY)

    @staticmethod
    def is_autonomous_active() -> bool:
        with _STATE_LOCK:
            return bool(_AUTONOMOUS_ACTIVE)

    @staticmethod
    def is_custom_sequence_active() -> bool:
        with _STATE_LOCK:
            return bool(_CUSTOM_SEQUENCE_ACTIVE)

    @staticmethod
    def begin_autonomous() -> tuple[bool, str]:
        global _AUTONOMOUS_ACTIVE
        with _STATE_LOCK:
            if _EMERGENCY:
                return False, "Emergency stop activo"
            if _AUTONOMOUS_ACTIVE:
                return False, "Modo autónomo ya activo"
            _AUTONOMOUS_ACTIVE = True
            return True, "Modo autónomo iniciado"

    @staticmethod
    def end_autonomous() -> None:
        global _AUTONOMOUS_ACTIVE
        with _STATE_LOCK:
            _AUTONOMOUS_ACTIVE = False

    @staticmethod
    def begin_custom_sequence() -> tuple[bool, str]:
        global _CUSTOM_SEQUENCE_ACTIVE
        with _STATE_LOCK:
            if _EMERGENCY:
                return False, "Emergency stop activo"
            if _AUTONOMOUS_ACTIVE:
                return False, "Modo autónomo activo"
            if _CUSTOM_SEQUENCE_ACTIVE:
                return False, "Secuencia personalizada ya activa"
            _CUSTOM_SEQUENCE_ACTIVE = True
            return True, "Secuencia personalizada iniciada"

    @staticmethod
    def end_custom_sequence() -> None:
        global _CUSTOM_SEQUENCE_ACTIVE
        with _STATE_LOCK:
            _CUSTOM_SEQUENCE_ACTIVE = False

    @staticmethod
    def blocked_for_operation() -> Optional[str]:
        with _STATE_LOCK:
            if _EMERGENCY:
                return "Emergency stop activo"
            if _AUTONOMOUS_ACTIVE:
                return "Modo autónomo activo"
        return None

    @staticmethod
    def blocked_for_calibration() -> Optional[str]:
        with _STATE_LOCK:
            if _EMERGENCY:
                return "Emergency stop activo"
            if _AUTONOMOUS_ACTIVE:
                return "Modo autónomo activo"
        return None

    @staticmethod
    def blocked_for_custom_sequence() -> Optional[str]:
        with _STATE_LOCK:
            if _EMERGENCY:
                return "Emergency stop activo"
            if _AUTONOMOUS_ACTIVE:
                return "Modo autónomo activo"
            if _CUSTOM_SEQUENCE_ACTIVE:
                return "Secuencia personalizada ya activa"
        return None

    @staticmethod
    def trigger_emergency(reason: str = "manual", safe=None, cerebro=None) -> None:
        global _EMERGENCY
        with _STATE_LOCK:
            _EMERGENCY = True

        coordinator_safe = safe
        coordinator_cerebro = cerebro
        if coordinator_safe is None or coordinator_cerebro is None:
            with _STATE_LOCK:
                if coordinator_safe is None:
                    coordinator_safe = _SAFE_CONTROLLER
                if coordinator_cerebro is None:
                    coordinator_cerebro = _CEREBRO

        log.critical("[EmergencyCoordinator] EMERGENCY STOP: %s", reason)

        if coordinator_safe is not None:
            try:
                coordinator_safe.emergency_stop()
                log.critical(
                    "[EmergencyCoordinator] SafeController: %s",
                    _safe_msg(coordinator_safe),
                )
            except Exception as exc:
                log.critical(
                    "[EmergencyCoordinator] SafeController falló: %s",
                    exc,
                )

        robot = None
        if coordinator_cerebro is not None:
            try:
                coordinator_cerebro.detener()
                log.critical("[EmergencyCoordinator] CerebroAutonomo detenido")
            except Exception as exc:
                log.critical(
                    "[EmergencyCoordinator] No se pudo detener CerebroAutonomo: %s",
                    exc,
                )
            try:
                robot = coordinator_cerebro.robot
            except Exception:
                robot = None

        if robot is not None:
            try:
                robot.controlador_servo.apagar_todos()
                log.critical("[EmergencyCoordinator] PWM autónomo cortado")
            except Exception as exc:
                log.critical(
                    "[EmergencyCoordinator] No se pudo cortar PWM autónomo: %s",
                    exc,
                )
            try:
                if robot.controlador_stepper:
                    robot.controlador_stepper.deshabilitar()
                    log.critical("[EmergencyCoordinator] Stepper deshabilitado")
            except Exception as exc:
                log.critical(
                    "[EmergencyCoordinator] No se pudo deshabilitar stepper: %s",
                    exc,
                )
            try:
                robot.resetear_tiempos()
            except Exception as exc:
                log.critical(
                    "[EmergencyCoordinator] No se pudo resetear tiempos: %s",
                    exc,
                )

    @staticmethod
    def reset_emergency(safe=None) -> tuple[bool, str]:
        global _EMERGENCY
        coordinator_safe = safe
        if coordinator_safe is None:
            with _STATE_LOCK:
                coordinator_safe = _SAFE_CONTROLLER

        if coordinator_safe is None:
            with _STATE_LOCK:
                _EMERGENCY = False
            return True, "No había SafeController registrado"

        was_emergency = False
        try:
            was_emergency = bool(coordinator_safe.is_emergency)
        except Exception:
            was_emergency = False

        if not was_emergency:
            with _STATE_LOCK:
                _EMERGENCY = False
            return True, "No había emergency stop activo"

        try:
            coordinator_safe.reset_emergency()
            with _STATE_LOCK:
                _EMERGENCY = False
            log.warning("[EmergencyCoordinator] Emergency stop reiniciado")
            return True, "Emergency stop reiniciado. Verificar posición del brazo."
        except Exception as exc:
            with _STATE_LOCK:
                _EMERGENCY = True
            log.critical("[EmergencyCoordinator] Reset falló: %s", exc)
            return False, f"Reset falló: {exc}"
