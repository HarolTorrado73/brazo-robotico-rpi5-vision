# REFERENCE - Brazo Robotico Con IA

Documento para que una IA o desarrollador futuro entienda el proyecto sin explorar todo el código.

## Propósito del proyecto

Brazo robotico autonomo que detecta objetos con YOLO, clasifica por color (HSV) y los deposita en recipientes ordenados. Interfaz web Flask con video en vivo.

## Hardware

- **Raspberry Pi 5**
- **PCA9685** (I2C 0x40): varios servos **posicionales ~180°**. La **web** usa `servo_config_legacy.json` (`tipo_servo`: `posicional_180`). **`ArmController`** usa `servo_config.json`: el **canal** de cada articulación es el que indiques en `joints.*.channel` (por defecto en repo: hombro **0**, codo **1**, pinza **3**, base **4** para no chocar con el cableado legacy 0–3).
- **TMC2208 + NEMA 17**: base horizontal (GPIO 17, 18, 19). Opcional: servo de base en un canal libre del PCA9685 (p. ej. **4**) con `ArmController` en lugar del NEMA
- **Arducam CSI**: rpicam-still / Picamera2

## Conexiones críticas

Ver **CONEXIONES.md**

## Flujo de ejecución

1. `autonomous_web.py` → Flask en :5000
2. `CerebroAutonomo` (autonomous_brain.py) orquesta: escanear → detectar → planificar → recoger → depositar
3. `ControladorRobotico` (robot_controller.py) ejecuta movimientos vía PCA9685 y TMC2208

## Archivos clave

| Archivo | Función |
|---------|---------|
| `arm_system/servo_config_legacy.json` | Pulsos calibrados para web/autónomo (neutral, hold, invertido, tiempos). **NO borrar** |
| `arm_system/servo_config.json` | **ArmController**: `joints` con `channel`, límites en **grados**, `pulse_*_us`, `home_sequence`, bloque `motion` |
| `config_sistema.py` | STEPPER_HABILITADO, CAMARA_HABILITADA, etc. |
| `arm_system/control/robot_controller.py` | ControladorServo (PCA9685), ControladorStepper (TMC2208), ControladorRobotico |
| `arm_system/control/arm_controller.py` | **ArmController** + **JointSpec**: solo PCA9685, ángulos absolutos, macros `open_gripper` / `move_base`, stub `move_to_target` (OpenCV futuro) |
| `test_motor.py` (raíz) | Prueba mínima: solo servo **base** con `ArmController` |

## `ArmController` (brazo por ángulos, OOP)

- **Propósito:** PCA9685 **solo servos**, sin lógica de otros actuadores; pensado para brazo estático 4 DOF (mapeo de canales editable en `servo_config.json`).
- **Importación** (raíz del repo en `PYTHONPATH`): `from arm_system.control.arm_controller import ArmController` (o `from arm_system.control import ArmController` con carga diferida en `control/__init__.py`).
- **API relevante:** `initialize_to_home_smooth()`, `set_joint_angle()`, `move_base` / `move_shoulder` / `move_elbow`, `open_gripper` / `close_gripper`, `sync_logical_angles()`, `release_all_pwm()`, `with ArmController() as arm:`.
- **Calibración:** edita `servo_config.json` (grados seguros y µs por articulación).
- **Convivencia:** `autonomous_web.py` y `CerebroAutonomo` siguen usando `robot_controller.py` + `servo_config_legacy.json` salvo que migres el código.

## Servos ~180° (v2 por defecto)

- Cada articulación tiene posición lógica **0…1**; se mapea a **pulso_min…pulso_max** (µs).
- **invertido**: invierte el sentido del 0…1 respecto al ancho de pulso.
- **Pinza**: interpola entre `pulso_cerrar` y `pulso_abrir`.
- **tiempo_max_***: tiempo de referencia para un recorrido completo en esa dirección (escala los pasos del modo autónomo). La calibración web los actualiza.
- Modo **continuo** (legacy): `tipo_servo: continuo` → misma lógica que el proyecto `main/definitivo`.

## Problemas conocidos

- **NEMA 17**: a veces no responde (cables, Vref, ENABLE). Alternativa: servo en canal 4
- **Cámara**: rpicam-still puede dar timeout. CameraManager intenta Picamera2 primero
- **Servos que caen**: aumentar compensación en pulso_hold (ver `servo_config_legacy.json`)

## Sustituir NEMA 17 por servo base

1. Añadir `"base"` en `servo_config_legacy.json` con canal 4 (y cablear) para el stack web, **o** usar `ArmController` + `servo_config.json` con base en el canal que elijas
2. En ControladorRobotico: no inicializar stepper, agregar servo 'base'
3. En api_mover (autonomous_web): joint='base' → controlador_servo.mover_por_tiempo
4. En autonomous_brain: cambiar angulo_base_pasos por tiempo o ángulo
