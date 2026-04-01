# Brazo robótico autónomo con IA (v2)

[![Python](https://img.shields.io/badge/Python-3.x-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![Flask](https://img.shields.io/badge/Flask-3.x-000000?logo=flask&logoColor=white)](https://flask.palletsprojects.com/)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.x-5C3EE8?logo=opencv&logoColor=white)](https://opencv.org/)
[![Ultralytics](https://img.shields.io/badge/Ultralytics-YOLO-111F68?logo=pytorch&logoColor=white)](https://docs.ultralytics.com/)
[![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi-5-C51A4A?logo=raspberrypi&logoColor=white)](https://www.raspberrypi.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

Brazo robótico autónomo que **detecta objetos con YOLO**, **clasifica por color (HSV)** y los deposita en **recipientes ordenados**. Incluye **interfaz web Flask** con video en vivo y **voz opcional** (micrófono USB + síntesis de voz).

> **Importante:** el repositorio no sustituye tu mesa de pruebas. Una instalación fiable exige validar en **tu** hardware: calibración, clases YOLO acordes a tus objetos, seguridad mecánica/eléctrica y audio. Consulta la checklist en **[PUESTA_EN_MARCHA.md](PUESTA_EN_MARCHA.md)**.

## Tabla de contenidos

- [Tecnologías](#tecnologías)
- [Requisitos previos](#requisitos-previos)
- [Instalación](#instalación)
- [Uso rápido](#uso-rápido)
- [Capturas (interfaz web)](#capturas-interfaz-web)
- [Hardware](#hardware)
- [Documentación](#documentación)
- [Estructura del repositorio](#estructura-del-repositorio)
- [Configuración](#configuración)
- [Voz opcional](#voz-opcional)
- [Licencia y comunidad](#licencia-y-comunidad)

## Tecnologías

| Área | Tecnologías |
|------|-------------|
| Lenguaje | Python 3 |
| Web | Flask |
| Visión | OpenCV (headless), Ultralytics YOLO, modelo NCNN en repo |
| Hardware (Pi) | Adafruit PCA9685, gpiozero, RPi.GPIO, Picamera2 (Linux) |
| Voz (opcional) | SpeechRecognition, espeak-ng / pyttsx3 (ver [HARDWARE_AUDIO.md](HARDWARE_AUDIO.md)) |

## Requisitos previos

- **Raspberry Pi 5** (recomendado) con Raspberry Pi OS, I2C y cámara habilitados.
- **Python 3** con `venv`; herramientas del sistema: `i2c-tools`, dependencias de cámara según [PUESTA_EN_MARCHA.md](PUESTA_EN_MARCHA.md).
- **Hardware:** PCA9685, servos, cámara CSI (p. ej. Arducam), fuente adecuada; opcional NEMA 17 + TMC2208 para la base.

En Windows puedes instalar dependencias “puras” de Python para editar código; el control de servos/cámara está pensado para **Linux en la Pi**.

## Instalación

```bash
sudo apt update
sudo apt install -y python3 python3-venv python3-pip i2c-tools
sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_camera 0

python3 -m venv venv --system-site-packages
source venv/bin/activate   # En Windows: venv\Scripts\activate
pip install -r requirements.txt
```

Voz (opcional): `sudo apt install -y espeak-ng portaudio19-dev alsa-utils` y `pip install -r requirements-voice.txt`. Detalle en [HARDWARE_AUDIO.md](HARDWARE_AUDIO.md).

## Uso rápido

```bash
cd arm_system
python autonomous_web.py
```

Abre en el navegador: `http://<IP_DE_LA_RASPBERRY>:5000` (incluye checklist de puesta en marcha y enlace a la guía en `/docs/puesta_en_marcha`).

**Prueba mínima del brazo por ángulos** (`ArmController`, solo en la Pi): desde la raíz del repositorio, `python3 test_motor.py`.

## Capturas (interfaz web)

La interfaz es visual (vídeo, controles de calibración y modo autónomo). Puedes añadir capturas al repositorio (por ejemplo en `docs/assets/`) y enlazarlas aquí:

```markdown
![Panel web](docs/assets/panel-web.png)
```

*(Aún no hay imágenes versionadas; sustituye la ruta cuando las añadas.)*

## Hardware

- **PCA9685** (I2C `0x40`): servos posicionales ~180° en canales 0–3 (mapeo clásico). La web usa `servo_config_legacy.json`; `ArmController` usa `servo_config.json` (canales en `joints.*.channel`).
- **TMC2208 + NEMA 17:** base horizontal (GPIO 17, 18, 19), opcional.
- **Arducam CSI:** `rpicam-still` / Picamera2.

Si usas **servos continuos**, en `arm_system/servo_config_legacy.json` pon `"tipo_servo": "continuo"` por articulación. Ver [REFERENCE.md](REFERENCE.md) y [CONEXIONES.md](CONEXIONES.md).

## Documentación

| Documento | Contenido |
|-----------|-----------|
| [PUESTA_EN_MARCHA.md](PUESTA_EN_MARCHA.md) | Checklist: calibración, YOLO, seguridad, audio |
| [RECIPIENTES_Y_LUZ.md](RECIPIENTES_Y_LUZ.md) | Cubetas, luz, objetos nuevos para YOLO |
| [CONEXIONES.md](CONEXIONES.md) | Diagrama de conexiones |
| [REFERENCE.md](REFERENCE.md) | Guía técnica para desarrollo |
| [LAB_WORKBENCH.md](LAB_WORKBENCH.md) | Laboratorio, modelo YOLO propio |
| [HARDWARE_AUDIO.md](HARDWARE_AUDIO.md) | Micrófono y altavoz en la Pi |

## Estructura del repositorio

```
arm_system/
├── autonomous_web.py         # Interfaz web principal (Flask)
├── autonomous_brain.py       # Lógica autónoma pick & place
├── config_sistema.py         # Configuración global
├── servo_config_legacy.json  # Pulsos calibrados web/autónomo (no borrar)
├── servo_config.json         # ArmController (grados + joints)
├── main.py                   # Menú consola alternativo
├── control/
│   ├── robot_controller.py   # ControladorRobotico (legacy)
│   └── arm_controller.py     # ArmController (PCA9685 por ángulos)
└── perception/vision/
    ├── camera/main.py
    ├── detection/            # YOLO
    └── color_detector.py
```

## Configuración

- `config_sistema.py`: `STEPPER_HABILITADO`, `CAMARA_HABILITADA`, `PERMITIR_DETECCION_SIMULADA` (`False` en brazo real para no usar datos ficticios si falla la visión; `True` solo para pruebas sin hardware).
- `servo_config_legacy.json`: `tipo_servo` (`posicional_180` o `continuo`), pulsos, tiempos, pinza.
- `servo_config.json`: calibración en **grados** para `ArmController` (ver [REFERENCE.md](REFERENCE.md)).

## Voz opcional

1. En la Raspberry Pi: `sudo apt install -y espeak-ng portaudio19-dev alsa-utils`
2. En el entorno virtual: `pip install -r requirements-voice.txt`
3. En `arm_system/config_sistema.py`: `VOZ_HABILITADA = True` (y opcional `VOZ_ANUNCIAR_EVENTOS`).
4. Si hay varios dispositivos de audio, ajusta `VOZ_MIC_DEVICE_INDEX` (ver [HARDWARE_AUDIO.md](HARDWARE_AUDIO.md)).

El reconocimiento usa la API en la nube de Google (SpeechRecognition); requiere **Internet**. Frases de ejemplo: «iniciar», «pausa», «escanear», «home», «emergencia», «calibrar servos», «calibrar color». Lista completa en `voice_assistant.py`.

Timbre “robot”: **espeak-ng** en Linux; sin eso, **pyttsx3**.

## Licencia y comunidad

- **Licencia:** [MIT](LICENSE)
- **Contribuir:** [CONTRIBUTING.md](CONTRIBUTING.md)
- **Código de conducta:** [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md)
- **Reportar vulnerabilidades:** [SECURITY.md](SECURITY.md)
