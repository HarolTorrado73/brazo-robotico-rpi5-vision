"""
hw_bus — Lock compartido del bus de hardware (I2C / PCA9685).

REGLA: Todo módulo que escriba al PCA9685 DEBE adquirir HW_LOCK antes de hacerlo.
       Este es el único punto de definición del lock; todos los módulos importan de aquí.

Módulos que usan HW_LOCK:
  - arm_system.safety.safe_controller  (SafeController — control manual por ángulos)
  - arm_system.control.robot_controller (ControladorServo — control autónomo por tiempo)

No hay lógica adicional en este módulo. Solo el lock.
"""

import threading

HW_LOCK: threading.Lock = threading.Lock()
