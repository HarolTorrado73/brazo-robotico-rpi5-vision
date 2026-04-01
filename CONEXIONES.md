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
| **4** | Libre (base servo si sustituyes NEMA 17) | **Base** (rotación), valor por defecto en el repo |

Los números de canal en `servo_config.json` deben coincidir con tus cables; si tu base va en otro pin, cambia solo `joints.base.channel`.

Si alternas scripts **sin re-cablear**, los movimientos serán incorrectos: ajusta cables o el campo `channel` en el JSON que uses.

### Bus I2C (común)

| PCA9685 | Conexión |
|---------|----------|
| VCC | 5V (Raspberry Pi) |
| GND | GND |
| SDA | GPIO 2 (Pin 3) |
| SCL | GPIO 3 (Pin 5) |

## TMC2208 (NEMA 17 - Base)

| TMC2208 | Raspberry Pi |
|---------|--------------|
| VM | 12V externa |
| GND | GND común |
| STEP | GPIO 17 (Pin 11) |
| DIR | GPIO 18 (Pin 12) |
| ENABLE | GPIO 19 (Pin 35) |

## Cámara

- Arducam CSI → Puerto CSI de la Pi
- Comandos: `rpicam-still`, `Picamera2`

## Alimentación

- 5V/20A para servos (PCA9685)
- 12V/5A para NEMA 17 + TMC2208
