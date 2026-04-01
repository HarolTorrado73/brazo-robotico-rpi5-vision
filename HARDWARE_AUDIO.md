# Audio en Raspberry Pi: micrófono, altavoz y checklist completo

La Raspberry Pi **no trae micrófono ni altavoz integrados** (salvo que uses un HAT o monitor con audio por HDMI). Para el asistente de voz del proyecto necesitas **entrada** (micrófono) y **salida** (parlante o auriculares).

---

## 1. Micrófono USB-C y la Pi (USB-A)

La mayoría de Pis expuestas al usuario tienen puertos **USB-A** (rectangulares). Si tu micrófono termina en **USB-C** (conector pequeño ovalado):

| Qué tienes | Qué necesitas |
|------------|----------------|
| Micrófono con **cable fijo USB-C macho** | Un adaptador **USB-C hembra → USB-A macho**. Enchufas el cable del micrófono al adaptador y el **USB-A macho** a la Raspberry. |
| Micrófono con **cable USB-A** | Conectar directo a la Pi (o a un hub USB alimentado si ya tienes muchos USB). |

**Importante**

- Usa un adaptador **solo pasivo** (datos + alimentación) de buena calidad; los muy baratos a veces fallan con audio USB.
- Si el micrófono es “para móvil” y trae **TRRS (4 anillos)** y no USB, no es el mismo caso: haría falta otra interfaz (tarjeta de sonido USB con entrada mic).
- **Hub USB alimentado** (con su propia fuente) ayuda si conectas cámara + micrófono + dongles: la Pi a veces no da suficiente corriente estable a todo a la vez.

---

## 2. Salida de sonido (para escuchar la voz del robot)

`espeak-ng` usa la **salida de audio por defecto** del sistema (PulseAudio / PipeWire en Raspberry Pi OS).

Opciones prácticas:

1. **Jack 3,5 mm** (Pi 4 / Pi 5): altavoces **activos** (con su propia alimentación USB o red) o auriculares. Evita altavoces pasivos “sin amplificador”: suenan muy bajos o no suenan.
2. **HDMI**: si el monitor tiene altavoces, el audio puede ir por HDMI; hay que elegir ese dispositivo como salida por defecto (ver abajo).
3. **USB**: dongle o altavoz USB “tarjeta de sonido”; Linux lo ve como segunda tarjeta de audio.
4. **Bluetooth**: posible, pero más pasos, latencia y cortes; no es lo primero que recomendaría en un robot industrial.

### Fijar salida por defecto (recomendado tras instalar hardware)

En escritorio: icono de volumen → elegir dispositivo de salida.

En consola (ejemplos):

```bash
# Listar tarjetas y dispositivos
aplay -l

# Probar altavoz (5 segundos, canal frontal)
speaker-test -t wav -c 2 -l 1
```

Si usas PulseAudio:

```bash
pactl list short sinks
pactl set-default-sink <NOMBRE_DEL_SINK>
```

---

## 3. Paquetes y Python (software)

En la Pi:

```bash
sudo apt update
sudo apt install -y espeak-ng portaudio19-dev alsa-utils
```

En el entorno virtual del proyecto:

```bash
pip install -r requirements-voice.txt
```

En `config_sistema.py`:

- `VOZ_HABILITADA = True` cuando el hardware esté probado.
- Si hay **varios** dispositivos de captura, lista índices con el script de abajo y pon `VOZ_MIC_DEVICE_INDEX = <número>`.

---

## 4. Pruebas obligatorias (antes de culpar al código)

### 4.1 ¿El sistema ve el micrófono?

```bash
arecord -l
```

Anota la tarjeta (`card X`) y dispositivo (`device Y`). Para PyAudio suele bastar el **índice** que lista el script:

```bash
cd arm_system
python3 -c "import pyaudio; p=pyaudio.PyAudio(); [print(i, p.get_device_info_by_index(i)['name']) for i in range(p.get_device_count()) if p.get_device_info_by_index(i)['maxInputChannels']>0]"
```

Si tu micrófono no aparece: cable, adaptador, hub o permisos (`sudo usermod -a -G audio $USER` y cerrar sesión).

### 4.2 Grabar y reproducir (prueba ciega del mic)

```bash
arecord -D plughw:1,0 -f cd -d 5 /tmp/prueba.wav   # cambia 1,0 según arecord -l
aplay /tmp/prueba.wav
```

Si el WAV está en silencio: volumen del mic (si tiene rueda), posición del mic, o dispositivo equivocado.

### 4.3 ¿Sale voz sintética?

```bash
espeak-ng -v es "Prueba de voz del brazo robótico"
```

Si no suena: revisa salida por defecto y volumen (`alsamixer` o control de volumen del escritorio).

### 4.4 Red

El reconocimiento actual usa **Google en la nube**. Sin Internet estable no habrá transcripción (el TTS local sí puede funcionar).

---

## 5. Configuración en este proyecto

| Archivo / variable | Uso |
|--------------------|-----|
| `config_sistema.py` → `VOZ_HABILITADA` | Activa el hilo de escucha al arrancar `autonomous_web.py`. |
| `VOZ_MIC_DEVICE_INDEX` | `None` = micrófono predeterminado; entero = índice PyAudio concreto. |
| `VOZ_ANUNCIAR_EVENTOS` | Si la web dispara también frases por altavoz. |
| `voice_assistant.py` | Lista de frases y mapeo a acciones. |

---

## 6. El proyecto “¿ya está hecho?” — qué falta en la práctica

Lo que **sí** está en el repo (v2): control del brazo, visión YOLO/color, modo autónomo pick & place, web, voz opcional con TTS + comandos por Google.

Lo que **tú** debes cerrar en el mundo real (no se puede “terminar” sin tu banco de pruebas):

1. **Calibración mecánica**: `servo_config_legacy.json` (web) / `servo_config.json` (`ArmController`), calibración de servos y color con tu iluminación.
2. **Cableado y alimentación**: servos/stepper con fuentes adecuadas; ruido eléctrico puede afectar USB/audio.
3. **Audio**: este documento + pruebas `arecord` / `espeak-ng`.
4. **Objetos y modelo YOLO**: clases que el modelo realmente detecta en tu mesa; si no coinciden, hay que cambiar dataset o modelo.
5. **Seguridad**: zona de trabajo, emergencia, nadie cerca cuando pruebas.
6. **Opcional futuro**: reconocimiento **offline** (Vosk/Whisper) si no quieres depender de Google; no viene instalado por defecto.

Nada de esto se “salta”: sin audio probado, la voz fallará; sin calibración, el brazo chocará o fallará el agarre.

---

## 7. Resumen de compra mínima sugerida

- Adaptador **USB-C hembra → USB-A macho** (si tu mic es USB-C).
- **Micrófono USB** con buena cancelación de ruido si el entorno es ruidoso (motores).
- **Altavoces activos** con entrada 3,5 mm **o** monitor con audio HDMI **o** tarjeta USB de audio + altavoz.
- (Opcional) **Hub USB 3.0 alimentado** si tienes muchos periféricos.

---

Si tras estas pruebas algo falla, anota: salida de `arecord -l`, `aplay -l`, el listado PyAudio y el mensaje de error del log al arrancar `autonomous_web.py`.

---

## 8. Contexto: checklist global del proyecto

El audio es **un cuarto** de la puesta en marcha real. El resto (calibración servos/color, YOLO, seguridad) está reunido en **[PUESTA_EN_MARCHA.md](PUESTA_EN_MARCHA.md)**.
