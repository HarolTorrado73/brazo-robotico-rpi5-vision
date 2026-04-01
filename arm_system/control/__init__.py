"""
Control del brazo: ``robot_controller`` (legado, web) y ``arm_controller`` (PCA9685 por ángulos).

La importación de ``ArmController`` es diferida para no cargar ``board``/I2C al usar solo el legado.
"""

from typing import Any

__all__ = ["ArmController", "JointSpec"]


def __getattr__(name: str) -> Any:
    if name == "ArmController":
        from .arm_controller import ArmController as _ArmController

        return _ArmController
    if name == "JointSpec":
        from .arm_controller import JointSpec as _JointSpec

        return _JointSpec
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
