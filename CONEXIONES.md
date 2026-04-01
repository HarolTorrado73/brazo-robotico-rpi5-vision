# Conexiones Hardware - Brazo Robotico

## PCA9685 (Servos ~180°)

Usa servos **NO continuos** (PWM posicional clásico). Alimentación 5 V adecuada al par (MG996R puede pedir pico alto; mejor fuente dedicada).

### Mapeo según controlador (mismo hardware, distinto JSON)

El cableado en los **canales 0–3** depende de si usas la pila **web/legacy** o **`ArmController`** (`servo_config.json`):

| Canal | Web + `ControladorServo` (`servo_config_legacy.json`) | `ArmController` (`servo_config.json`, **editable**) |
|-------|--------------------------------------------------------|------------------------------------------------------|
| **0** | Hombro (shoulder) | Hombro (shoulder) |
| **1** | Codo (elbow) | Codo (elbow) |
| **2** | Muñeca (wrist) | *(no usado en el JSON por defecto del repo; puedes asignar una articulación aquí)* |
| **3** | Pinza (gripper) | Pinza (gripper) |
| **4** | **Base** (rotación horizontal, MG996R ~180°) | **Base** (rotación), mismo canal en `servo_config.json` → `joints.base` |

Los números de canal en `servo_config.json` deben coincidir con tus cables; si tu base va en otro pin, cambia solo `joints.base.channel`.

Si alternas scripts **sin re-cablear**, los movimientos serán incorrectos: ajusta cables o el campo `channel` en el JSON que uses.

### Bus I2C (común)

| PCA9685 | Conexión |
|---------|----------|
| VCC | 5V (Raspberry Pi) |
| GND | GND |
| SDA | GPIO 2 (Pin 3) |
| SCL | GPIO 3 (Pin 5) |

## Base (MG996R)

La rotación de la base se hace con un **servo estándar ~180°** (p. ej. **MG996R**) en el **canal 4** del PCA9685, con calibración en `servo_config_legacy.json` → clave `"base"`.

Opcional (avanzado): si en `config_sistema.py` pones `STEPPER_HABILITADO = True`, puedes usar un **TMC2208 + NEMA 17** en GPIO 17 (STEP), 18 (DIR) y 19 (ENABLE) en lugar del servo de base; no es el montaje documentado por defecto.

## Cámara

- Arducam CSI → Puerto CSI de la Pi
- Comandos: `rpicam-still`, `Picamera2`

## Alimentación

- 5 V con **corriente suficiente** para todos los servos (PCA9685); el MG996R puede tener picos altos: mejor fuente dedicada o al menos no alimentar solo desde el 5 V del pin de la Pi si el brazo es exigente.
- Si usas montaje opcional con **NEMA 17 + TMC2208**, añade alimentación **12 V** adecuada al motor y driver.
