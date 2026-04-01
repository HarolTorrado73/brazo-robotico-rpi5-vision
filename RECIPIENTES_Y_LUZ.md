# Recipientes, luz y objetos “nuevos” para el brazo

El sistema detecta **recipientes** como **regiones grandes y de color sólido** (HSV), no por forma de “cubo” concreta. Así puedes diseñarlos para que sean fiables incluso con luz incómoda — dentro de lo razonable.

---

## 1. Cómo debe ser el recipiente

| Criterio | Recomendación |
|----------|----------------|
| **Color** | Superficies **mate** (plástico opaco pintado), **un solo color** fuerte: rojo, azul, verde, amarillo, naranja o morado (los que define `color_detector` / `COLORES_RECIPIENTES`). |
| **Brillo** | Evita **metal pulido**, **acrílico transparente** o **blanco muy reflejante**: actúan como espejo y rompen el HSV. |
| **Tamaño en imagen** | Debe ocupar **área grande** en la foto (el código usa `area_minima` alto al buscar recipientes). Cuanto más pequeño el cubo en pantalla, más difícil superar el umbral. |
| **Forma** | Cualquier forma con **proporción razonable** (no una línea fina): el filtro descarta contornos muy alargados. |
| **Altura y boca** | Altura compatible con que la pinza **suelte dentro** sin chocar el borde; boca **más ancha** que el objeto típico. |
| **Contraste con la mesa** | Mesa **neutra** (gris mate, negro mate, corcho): si el recipiente y la mesa comparten tono, el blob de color se confunde. |
| **Posición** | Fijos entre ciclos; que entren bien en el encuadre de la cámara en posición de **escaneo**. |

**Manipulación del entorno (luz extrema y reflejos):** no “anula” la física, pero **sí** mejora mucho el resultado:

- **Luz difusa**: LED con **difusor** o pantalla, o luz indirecta; evita un foco puntual justo encima del objeto creando brillo puntual.
- **Menos reflejos**: inclinar ligeramente recipientes o cambiar ángulo de cámara para que no se vea el foco reflejado en el plástico.
- **Calibrar color** en la web **con la misma luz** que usarás al trabajar (`Calibrar color`).
- Si hace falta, baja un poco la luz directa sobre la zona y sube luz ambiental uniforme.

---

## 2. Objetos que el modelo YOLO nunca vio

La visión **no adivina** clases nuevas: si el objeto no está en el entrenamiento del modelo, YOLO puede:

- **no detectarlo**,
- o **confundirlo** con otra clase.

**Qué hacer:**

1. **Entrenar o fine-tune** un YOLO con fotos de tus piezas (laboratorio, cajas, etc.) y sustituir el modelo (ver `LAB_WORKBENCH.md`).
2. Mapear clases → recipiente con `YOLO_LAB_CLASE_A_COLOR` en `config_sistema.py`.
3. Ajustar **`CONFIANZA_MINIMA_DETECCION`** y, si hace falta, **`imgsz`** en `autonomous_brain.py` (más resolución = más carga en la Pi).

El color HSV en el recorte de la caja **sigue ayudando** a clasificar el recipiente aunque la clase YOLO sea genérica, pero **no sustituye** una detección estable del bounding box.

---

## 3. Modo sin detección simulada (`PERMITIR_DETECCION_SIMULADA`)

En `config_sistema.py`:

- **`False` (recomendado con brazo real):** si falla la cámara, YOLO o la detección de recipientes, **no** se inventan manzanas/botellas ni cubos en posiciones ficticias; el autónomo **no ejecuta** pick & place basado en datos falsos. Si hay objetos pero **ningún** recipiente visible, **no** se genera plan hacia un “recipiente por defecto” inventado.
- **`True`:** útil en **PC sin hardware** para ver el flujo del programa; **no** usar en el robot con personas cerca.

---

## 4. Si los recipientes no aparecen en el escaneo

- Sube el tamaño aparente del recipiente en la imagen o acerca la cámara.
- Comprueba que el color esté en los rangos HSV de `color_detector` / `COLORES_RECIPIENTES`.
- En código, la llamada usa `area_minima=3000` en `_detectar_recipientes`; si tus cubos son pequeños en imagen, habría que **bajar** ese valor (con cuidado: más ruido).

---

Resumen: **recipiente mate, color vivo, grande en imagen, mesa contrastada, luz difusa y calibración**. Eso es lo que más se acerca a “manipular el entorno” de forma realista; el software ayuda con CLAHE y offsets, pero no sustituye un buen montaje físico.
