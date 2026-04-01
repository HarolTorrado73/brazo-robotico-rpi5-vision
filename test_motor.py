#!/usr/bin/env python3
"""
Prueba mínima del servo de BASE con ArmController (PCA9685, I2C).

Ejecutar en la Raspberry Pi desde la raíz del proyecto:
    python3 test_motor.py

Solo mueve el canal configurado como ``base`` en arm_system/servo_config.json
(ida y vuelta a ángulos seguros). El resto de articulaciones no recibe comandos aquí.
"""

from __future__ import annotations

import logging
import os
import sys

# Raíz del repositorio importable como paquete ``arm_system``
_ROOT = os.path.dirname(os.path.abspath(__file__))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

from arm_system.control.arm_controller import ArmController


def main() -> None:
    arm: ArmController | None = None
    try:
        arm = ArmController()
        base = arm.joints[ArmController.JOINT_BASE]
        home = float(base.angle_home_deg)
        lo = float(base.angle_safe_min_deg) + 15.0
        hi = float(base.angle_safe_max_deg) - 15.0
        # Desplazamiento moderado dentro del rango seguro (ida y vuelta)
        forward = min(home + 25.0, hi)
        if forward <= home:
            forward = max(home - 25.0, lo)
        logging.info("Prueba BASE: home=%.1f° -> %.1f° -> %.1f°", home, forward, home)
        arm.move_base(forward, smooth=True)
        arm.move_base(home, smooth=True)
        logging.info("Prueba BASE finalizada correctamente.")
    finally:
        if arm is not None:
            arm.close()


if __name__ == "__main__":
    main()
