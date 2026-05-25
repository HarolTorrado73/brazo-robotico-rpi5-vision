# Plataforma robótica modular con visión artificial asistida

[![Python](https://img.shields.io/badge/Python-3.x-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![Flask](https://img.shields.io/badge/Flask-3.x-000000?logo=flask&logoColor=white)](https://flask.palletsprojects.com/)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.x-5C3EE8?logo=opencv&logoColor=white)](https://opencv.org/)
[![Ultralytics](https://img.shields.io/badge/Ultralytics-YOLO-111F68?logo=pytorch&logoColor=white)](https://docs.ultralytics.com/)
[![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi-5-C51A4A?logo=raspberrypi&logoColor=white)](https://www.raspberrypi.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

## Visión general

Este proyecto es una **plataforma robótica modular con visión artificial y automatización física asistida** sobre hardware real.

Su foco principal es integrar:

- percepción visual básica funcional,
- lógica basada en reglas para clasificación y manipulación,
- control de servomotores a través de PCA9685,
- una interfaz de supervisión web con Flask,
- mecanismos de seguridad para operaciones en Raspberry Pi.

## Qué hace el sistema

El proyecto permite:

- detectar objetos con un modelo YOLO preentrenado,
- clasificar colores usando análisis HSV,
- ejecutar movimientos secuenciales predefinidos para manipular un brazo robótico,
- controlar servos físicos reales a través de PCA9685,
- supervisar estado y vídeo en una interfaz web,
- aplicar protecciones de seguridad para evitar daños en el hardware.

## Qué no hace el sistema

Este repositorio no es una solución de inteligencia artificial avanzada. No incluye:

- aprendizaje autónomo complejo,
- adaptación continua del modelo en runtime,
- entrenamiento de redes neuronales desde cero con un dataset propio robusto,
- planeación robótica cinemática avanzada,
- SLAM,
- reinforcement learning,
- autonomía total sin supervisión.

## Tecnologías y componentes

- Raspberry Pi 5
- Python 3
- Flask
- OpenCV
- Ultralytics YOLO (preentrenado)
- PCA9685
- Servomotores
- Cámara CSI
- Análisis HSV para clasificación de color

## Arquitectura por módulos

### 1. Percepción

- `arm_system/perception/vision/camera/main.py`
  - captura imagen con Picamera2, `rpicam-still`, `libcamera-still` o OpenCV.
- `arm_system/perception/vision/detection/main.py`
  - carga modelo YOLO preentrenado con `ModelLoader`.
- `arm_system/perception/vision/color_detector.py`
  - analiza regiones en HSV para determinar el color dominante y localizar recipientes.

### 2. Decisión y automatización asistida

- `arm_system/autonomous_brain.py`
  - orquesta el ciclo de escaneo, detección, clasificación y ejecución de tareas.
  - usa lógica basada en reglas y secuencias predefinidas.
  - incluye reintentos y manejos de fallo para no actuar con datos poco fiables.

### 3. Actuación y control de hardware

- `arm_system/control/arm_controller.py`
  - control de servos basado en ángulos mediante `servo_config.json`.
  - convierte ángulos seguros en pulsos PWM para PCA9685.
- `arm_system/control/robot_controller.py`
  - controlador legacy que usa `servo_config_legacy.json` y lógica por tiempo/pulsos.

### 4. Interfaz y supervisión

- `arm_system/autonomous_web.py`
  - servidor Flask que muestra vídeo, estado, diagnósticos y controles.
  - permite iniciar el ciclo asistido y controlar el brazo manualmente.

## Seguridad y robustez

Este proyecto prioriza la seguridad de hardware y la operación robusta:

- `SafeController` (en `arm_system/safety/safe_controller.py`)
  - valida ángulos objetivos antes de mover servos,
  - aplica rate limiting,
  - evita saltos bruscos por articulación,
  - realiza interpolación suave con pasos controlados,
  - ofrece `emergency_stop()` y `reset_emergency()`.

- `HW_LOCK` (en `arm_system/hw_bus.py`)
  - bloqueo global que evita accesos simultáneos al PCA9685,
  - asegura coexistencia entre modo manual y subsistemas autónomos,
  - reduce el riesgo de comandos conflictivos en I2C.

- `servo_config.json`
  - define límites seguros por articulación,
  - describe `pulse_min_us`, `pulse_max_us`, `angle_safe_min_deg`, `angle_safe_max_deg` y `angle_home_deg`.

## Integración hardware/software

El valor real del proyecto está en integrar componentes físicos y lógicos:

- Cámara CSI con captura de imagen real.
- Detección visual con YOLO preentrenado.
- Clasificación de color basada en HSV.
- Control de servos con PCA9685.
- Interfaz web para monitoreo y control.
- Seguridad al mover hardware real.

Esto no es solo software: es una plataforma embebida que coordina percepción y movimiento físico.

## Instalación rápida

```bash
sudo apt update
sudo apt install -y python3 python3-venv python3-pip i2c-tools
sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_camera 0

python3 -m venv venv --system-site-packages
source venv/bin/activate
pip install -r requirements.txt
```

Para voz (opcional):

```bash
sudo apt install -y espeak-ng portaudio19-dev alsa-utils
pip install -r requirements-voice.txt
```

## Uso inicial

```bash
cd ~/Downloads/BrazoRoboticoConIA/BrazoRoboticoConIA-v2
source venv/bin/activate
python3 -m arm_system.autonomous_web
```

Abrir en el navegador:

```text
http://<IP_DE_LA_RASPBERRY>:5000
```

## Qué presentar

Presenta el proyecto como una **plataforma robótica asistida por visión** con automatización física limitada, no como un robot inteligente autónomo.

### Mensaje recomendado

> El proyecto es una plataforma robótica modular con visión artificial y automatización física asistida. Usa modelos YOLO preentrenados para percepción visual y reglas de control para mover el brazo de forma segura.

### Qué enfatizar

- integración hardware/software,
- modularidad del diseño,
- seguridad de movimiento real,
- separación clara entre percepción, decisión y actuación,
- uso práctico de Raspberry Pi con cámara y PCA9685.

### Qué evitar

- afirmar que el sistema aprende solo,
- venderlo como IA avanzada o autonomía total,
- sugerir que es un sistema industrial inteligente completo.

## Limitaciones reales

- depende de un modelo YOLO preentrenado, no de un entrenamiento propio,
- la detección funciona mejor en condiciones controladas,
- la automatización es secuencial y basada en reglas,
- no hay aprendizaje adaptativo ni planificación cinemática avanzada,
- el sistema necesita calibración física para cada instalación.

## Mejoras futuras realistas

- entrenar un modelo YOLO propio con datos del entorno real,
- mejorar la detección de objetos pequeños ajustando `imgsz` y rangos HSV,
- añadir sensores físicos de contacto o fuerza,
- refinar la planificación de trayectorias con cinemática inversa,
- documentar más casos de calibración y pruebas.

## Estructura del repositorio

```
arm_system/
├── autonomous_web.py         # Interfaz web principal (Flask)
├── autonomous_brain.py       # Ciclo de automatización asistida
├── config_sistema.py         # Configuración global del sistema
├── servo_config_legacy.json  # Pulsos calibrados para modo legacy
├── servo_config.json         # Calibración en grados para ArmController
├── main.py                   # Menú de consola alternativo
├── control/
│   ├── arm_controller.py     # Control moderno de servos por ángulos
│   └── robot_controller.py   # Control legacy de servos por tiempo/pulsos
├── perception/
│   └── vision/
│       ├── camera/main.py
│       ├── detection/main.py
│       └── color_detector.py
├── safety/
│   └── safe_controller.py    # Control seguro de servos
├── hw_bus.py                 # Lock global para PCA9685
└── voice_assistant.py        # Soporte opcional de voz
```

## Documentación adicional

- [PUESTA_EN_MARCHA.md](PUESTA_EN_MARCHA.md) — checklist de seguridad y calibración.
- [REFERENCE.md](REFERENCE.md) — guía técnica para parámetros y hardware.
- [LAB_WORKBENCH.md](LAB_WORKBENCH.md) — recomendaciones para entrenar modelos propios.
- [HARDWARE_AUDIO.md](HARDWARE_AUDIO.md) — audio y micrófono en la Pi.

## Licencia y comunidad

- Licencia MIT.
- Normas de contribución en [CONTRIBUTING.md](CONTRIBUTING.md).
- Código de conducta en [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md).
- Reporte de seguridad en [SECURITY.md](SECURITY.md).

---

Para mayor profundidad en la presentación y los argumentos técnicos, consulta `docs/PRESENTATION_GUIDE.md`. 
