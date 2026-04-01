# Brazo Robotico Autonomo v2

Proyecto limpio del brazo robotico con visión artificial. Pick & place autónomo con detección YOLO y clasificación por color.

### Importante: el repo no sustituye tu mesa de pruebas

El software incluye brazo, visión, web y voz opcional, pero **una instalación “terminada”** exige validar en **tu** hardware: calibración, clases YOLO acordes a tus objetos, seguridad mecánica/eléctrica y audio probado. Lee la checklist completa aquí: **[PUESTA_EN_MARCHA.md](PUESTA_EN_MARCHA.md)**.

## Inicio rápido

```bash
cd arm_system
python autonomous_web.py
```

Abrir en navegador: **http://&lt;IP_RASPBERRY&gt;:5000** (incluye checklist de puesta en marcha y enlace a la guía en `/docs/puesta_en_marcha`)

## Hardware

- **Raspberry Pi 5**
- **4 servos estándar ~180°** (ej. MG996R / SG90 en modo angular, no continuos) vía **PCA9685** canales 0–3
- **NEMA 17 + TMC2208** para base horizontal (opcional, GPIO 17, 18, 19)
- **Arducam CSI** para visión

Si aún tienes **servos continuos**, en `arm_system/servo_config_legacy.json` pon `"tipo_servo": "continuo"` en cada articulación (comportamiento tipo `BrazoRoboticoConIA-main`). El archivo `arm_system/servo_config.json` es para **`ArmController`** (ángulos); prueba de base: `python3 test_motor.py` desde la raíz del repo (en la Pi).

## Documentación

- **[PUESTA_EN_MARCHA.md](PUESTA_EN_MARCHA.md)** – Checklist real: calibración, YOLO, seguridad, audio (lo que el código solo no puede cerrar)
- **[RECIPIENTES_Y_LUZ.md](RECIPIENTES_Y_LUZ.md)** – Diseño de cubetas/colores, luz y reflejos; objetos nuevos para YOLO; modo sin simulación
- **[CONEXIONES.md](CONEXIONES.md)** – Diagrama de conexiones hardware
- **[REFERENCE.md](REFERENCE.md)** – Guía técnica para desarrollo e IA
- **[LAB_WORKBENCH.md](LAB_WORKBENCH.md)** – Usar el brazo con herramientas y componentes de laboratorio (cautín, resistencias, modelo YOLO propio)
- **[HARDWARE_AUDIO.md](HARDWARE_AUDIO.md)** – Micrófono, altavoz y pruebas de audio en la Pi

## Estructura

```
arm_system/
├── autonomous_web.py    # Interfaz web principal (Flask)
├── autonomous_brain.py  # Lógica autónoma pick & place
├── config_sistema.py    # Configuración global
├── servo_config_legacy.json  # Pulsos calibrados web/autónomo (NO borrar)
├── servo_config.json         # ArmController (grados + joints)
├── main.py              # Menú consola alternativo
├── control/
│   ├── robot_controller.py   # ControladorRobotico (legacy)
│   └── arm_controller.py     # ArmController (PCA9685 por ángulos)
└── perception/vision/
    ├── camera/main.py
    ├── detection/       # YOLO
    └── color_detector.py
```

## Instalación (Raspberry Pi)

```bash
sudo apt update
sudo apt install -y python3 python3-venv python3-pip i2c-tools
sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_camera 0

python3 -m venv venv --system-site-packages
source venv/bin/activate
pip install -r requirements.txt
```

## Configuración

- `config_sistema.py`: `STEPPER_HABILITADO`, `CAMARA_HABILITADA`, `PERMITIR_DETECCION_SIMULADA` (`False` en brazo real: no inventa objetos/recipientes si falla la visión; `True` solo para pruebas sin hardware)
- `servo_config_legacy.json`: `tipo_servo` (`posicional_180` o `continuo`), `pulso_min`/`pulso_max`, tiempos de recorrido, pinza abrir/cerrar
- `servo_config.json`: calibración por **grados** para `ArmController` (ver [REFERENCE.md](REFERENCE.md))

## Voz (micrófono USB + respuesta robotizada)

**Guía completa de hardware (USB-C, adaptadores, altavoces, pruebas paso a paso):** [HARDWARE_AUDIO.md](HARDWARE_AUDIO.md)

1. En la Raspberry Pi: `sudo apt install -y espeak-ng portaudio19-dev alsa-utils`
2. En el venv: `pip install -r requirements-voice.txt`
3. En `arm_system/config_sistema.py` pon `VOZ_HABILITADA = True` (y opcional `VOZ_ANUNCIAR_EVENTOS` para que la web también dispare frases por altavoces).
4. Si tienes varios dispositivos de audio, fija `VOZ_MIC_DEVICE_INDEX` (ver HARDWARE_AUDIO.md).
5. Conecta **micrófono USB** y **salida de audio** (jack 3,5 mm, HDMI o USB); la Pi no trae micrófono ni altavoz integrados.

**Reconocimiento:** usa la API en la nube de Google (SpeechRecognition); hace falta **Internet**. El código ignora errores de mic/red y sigue funcionando el resto del sistema.

**Timbre “robot”:** en Linux se usa **espeak-ng** (voz metálica). Sin espeak, cae a **pyttsx3** (menos caracter robot).

**Frases de ejemplo (español):** «iniciar», «pausa», «reanudar», «detener», «escanear», «home», «emergencia», «calibrar servos», «calibrar color». Ver `voice_assistant.py` para la lista completa.
