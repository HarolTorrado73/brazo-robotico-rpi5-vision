# Puesta en marcha real: lo que el repositorio no puede “terminar” por ti

El código en este repo es **software listo para integrar**, pero un brazo robótico **no está al 100 %** hasta que lo validas **en tu mesa**, con **tu** luz, **tus** objetos y **tu** cableado. Esto no es un fallo del proyecto: es inherente a cualquier sistema físico.

Usa este documento como **lista de comprobación** antes de confiar en modo autónomo o en voz.

**En la interfaz web** (`autonomous_web.py`): al final del panel hay un **checklist interactivo** (casillas guardadas en el navegador con `localStorage`) y un enlace a **Guía completa** que abre este mismo contenido en `/docs/puesta_en_marcha`.

---

## 1. Calibración de servos y color (obligatorio en hardware real)

### Dos archivos JSON de servos (no confundirlos)

| Archivo | Uso | Formato |
|---------|-----|---------|
| **`arm_system/servo_config_legacy.json`** | Interfaz web, modo autónomo, `ControladorServo` | Pulsos µs, posición 0…1, `tiempo_max_*`, hombro/codo/muñeca/pinza en **0–3**, **base MG996R** en canal **4** |
| **`arm_system/servo_config.json`** | `ArmController`, scripts como `test_motor.py` | `joints` en **grados** + `channel` por articulación (ej. hombro 0, codo 1, pinza 3, base **4** si 0–3 ya están usados) |

La web **no** usa `ArmController` por defecto. Si tu montaje físico sigue el mapeo **legacy** (canal 0 = hombro), no ejecutes `ArmController`/`test_motor.py` sin alinear canales en el JSON o el cableado. Detalle de pines: [CONEXIONES.md](CONEXIONES.md).

**Ejemplo mínimo con `ArmController` (solo Pi, consola):**

```python
import logging
logging.basicConfig(level=logging.INFO)
from arm_system.control.arm_controller import ArmController

with ArmController() as arm:
    arm.initialize_to_home_smooth()
    arm.open_gripper()
    arm.move_base(75)
```

**Prueba rápida solo base:** desde la raíz del repo, `python3 test_motor.py` (ida y vuelta en el servo de base; cierra I2C en `finally`).

Tras el primer arranque, si el brazo **no** estaba físicamente en home, usa `sync_logical_angles({...})` o pasa `assumed_positions_deg` a `initialize_to_home_smooth` (docstring de la clase).

| Paso | Qué hacer | Dónde / cómo |
|------|-----------|----------------|
| 1.1 | Revisar calibración: web/legacy vs `ArmController` según el JSON que toques | `servo_config_legacy.json` (web) y/o `servo_config.json` (ángulos) |
| 1.2 | **Calibrar servos** con topes reales (recorrido, pinza) | Interfaz web → **Calibrar servos** (o rutina equivalente en `robot_controller`) |
| 1.3 | Probar **HOME** y movimientos manuales: sin vibración excesiva, sin forzar mecánica | Web → controles manuales |
| 1.4 | Con la **misma iluminación** que usarás en producción, pulsar **Calibrar color** | Web → **Calibrar color** (ajusta offsets HSV en `color_detector`) |
| 1.5 | Colocar **recipientes de colores** fijos y repetir escaneo: que la lista de recipientes sea estable | Web → **Escanear** |
| 1.6 | Con brazo real: `PERMITIR_DETECCION_SIMULADA = False` en `config_sistema.py` para no mover el brazo con datos ficticios si falla YOLO/cámara | Ver [RECIPIENTES_Y_LUZ.md](RECIPIENTES_Y_LUZ.md) |

**Si saltas esto:** el brazo puede ir a ángulos incorrectos, fallar el agarre o clasificar mal el color → parecerá “que falla la IA” cuando es **calibración**.

**Recipientes y luz:** diseño de cubetas, reflejos y objetos nuevos para el modelo → [RECIPIENTES_Y_LUZ.md](RECIPIENTES_Y_LUZ.md).

---

## 2. YOLO y las clases que tú usas

| Paso | Qué hacer | Notas |
|------|-----------|--------|
| 2.1 | Confirmar qué clases predice el modelo actual (COCO: manzana, botella, tijeras, etc.) | Ver `metadata.yaml` del modelo o prueba visual en la web |
| 2.2 | Si tus piezas son **otras** (laboratorio, cajas propias): entrenar **YOLO propio** y sustituir el modelo | Guía: [LAB_WORKBENCH.md](LAB_WORKBENCH.md) |
| 2.3 | Rellenar **`YOLO_LAB_CLASE_A_COLOR`** en `config_sistema.py` con nombres **exactos** de clase → color de recipiente | Solo aplica con modelo entrenado por ti |
| 2.4 | Ajustar **`CONFIANZA_MINIMA_DETECCION`** si hay falsos positivos o no detecta nada | `config_sistema.py` |
| 2.5 | (Opcional) Subir resolución de inferencia **`imgsz`** en la llamada a `predict` si las piezas son muy pequeñas | `autonomous_brain.py` — más carga en la Pi |

**Si saltas esto:** el sistema “no ve” tus objetos o los confunde con clases COCO irrelevantes.

---

## 3. Mecánica, alimentación y seguridad

| Paso | Qué hacer |
|------|-----------|
| 3.1 | **Fuentes** adecuadas: servos y motor paso a paso con **corriente suficiente**; masa común correcta; evita alimentar servos solo desde el 5 V de la Pi si el brazo es exigente |
| 3.2 | **Zona de trabajo despejada**: personas, cables sueltos y objetos frágiles fuera del alcance del brazo |
| 3.3 | Probar **parada de emergencia** en la web y conocer el comportamiento (servos apagados; si usas stepper opcional, driver deshabilitado) |
| 3.4 | Primera vez: movimientos **lentos**, velocidad autónoma conservadora (`VELOCIDAD_AUTONOMA` en `config_sistema.py`) |
| 3.5 | Cableado revisado: I2C PCA9685, **base MG996R en canal 4**, CSI cámara — según [CONEXIONES.md](CONEXIONES.md) |

**Si saltas esto:** riesgo de daños mecánicos, picos de corriente o movimientos impredecibles.

---

## 4. Audio (micrófono y altavoz)

La Pi **no** trae micrófono ni parlante. Si activas voz y “no funciona”, en muchos casos el fallo es **dispositivo o salida por defecto**, no el código Python.

| Paso | Qué hacer | Detalle |
|------|-----------|---------|
| 4.1 | Completar pruebas **`arecord` / `aplay` / `espeak-ng`** | [HARDWARE_AUDIO.md](HARDWARE_AUDIO.md) |
| 4.2 | Si hay varios micrófonos, fijar **`VOZ_MIC_DEVICE_INDEX`** | `config_sistema.py` |
| 4.3 | Confirmar **Internet** si usas reconocimiento Google | Sin red, no hay STT en la nube |
| 4.4 | Instalar dependencias: `requirements-voice.txt` + `espeak-ng`, `portaudio19-dev`, `alsa-utils` | README sección Voz |

---

## 5. Orden sugerido el primer día

1. Cableado y alimentación (apartado 3).  
2. Arranque web, vídeo, movimiento manual y HOME (apartado 1, sin autónomo).  
3. Calibrar servos y color (apartado 1).  
4. Escanear con objetos de prueba que el YOLO **sí** conozca; luego migrar a tu modelo/clases (apartado 2).  
5. Si usas voz: audio apartado 4 **antes** de depurar comandos.

---

## 6. Raspberry Pi OS: `apt upgrade` y error de initramfs

Tras un **`sudo apt upgrade`**, en algunas instalaciones aparece:

- `mkinitramfs: failed to determine device for /`
- `update-initramfs: failed for /boot/initrd.img-...`
- `dpkg` deja paquetes a medias (`linux-image-*`, `initramfs-tools`, etc.)

**No es un fallo del software de este repositorio**, sino de **`initramfs-tools`** con `MODULES=dep` (solo incluye módulos “necesarios” y a veces no resuelve bien el disco raíz al generar el initrd).

En una Pi típica, **`/`** está en **`/dev/mmcblk0p2`** (`ext4`); eso es normal. Solución habitual:

1. Forzar **`MODULES=most`** (initramfs más grande pero generación fiable):
   ```bash
   echo 'MODULES=most' | sudo tee /etc/initramfs-tools/conf.d/zz-modules-most.conf
   ```
   *(Alternativa: en `/etc/initramfs-tools/initramfs.conf` cambiar `MODULES=dep` por `MODULES=most`.)*

2. Regenerar y cerrar el estado de paquetes:
   ```bash
   sudo update-initramfs -u -k all
   sudo dpkg --configure -a
   sudo apt -f install
   ```

3. Si todo termina sin error: `sudo reboot`.

Si sigue fallando, conserva la salida de `findmnt /`, `lsblk -f` y `grep -r ^MODULES /etc/initramfs-tools/` para diagnosticar (foros Raspberry Pi / Debian).

---

## 7. Documentos relacionados

| Documento | Contenido |
|-----------|-----------|
| [CONEXIONES.md](CONEXIONES.md) | Esquema eléctrico / pines (mapeo legacy vs `ArmController`) |
| [REFERENCE.md](REFERENCE.md) | Detalles técnicos de desarrollo |
| [LAB_WORKBENCH.md](LAB_WORKBENCH.md) | Laboratorio, YOLO propio, resistencias, cautín |
| [HARDWARE_AUDIO.md](HARDWARE_AUDIO.md) | USB-C, altavoces, ALSA, pruebas |

Cuando los cuatro bloques (1–4) estén tachados en **tu** entorno, sí puedes decir que **tu instalación** del proyecto está operativa; el repo en sí sigue siendo una base — la “versión cerrada” es la que **tú** validas en banco.
