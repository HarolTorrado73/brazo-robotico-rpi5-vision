# Mesa de laboratorio: cautín, resistencias, herramientas (no “objetos de cocina”)

El YOLO que trae el proyecto (**YOLO11s en COCO**) está entrenado para cosas como personas, botellas, manzanas, teclados, tijeras, etc. **No** incluye clases útiles para:

- **Cautín** (soldador)
- **Resistencias** sueltas (muy pequeñas en imagen)
- **Condensadores**, **pinzas**, **protoboard**, **estaño**, etc.

Por tanto, **no puedes esperar** que el modelo actual “entienda” el taller solo cambiando texto en el README: hace falta **visión adaptada a tu mesa** y, en muchos casos, **un modelo entrenado por ti**.

---

## 1. Qué hace hoy el software

1. **YOLO** dibuja cajas alrededor de lo que reconoce (clases COCO).
2. **Color HSV** en esa caja decide el **recipiente** (rojo / azul / verde…).

Para laboratorio tienes dos caminos:

| Enfoque | Idea | Esfuerzo |
|--------|------|----------|
| **A. Modelo propio (recomendado)** | Entrenas YOLO con fotos de *tus* herramientas y componentes; exportas a NCNN/ONNX como ya hace el proyecto. | Medio: dataset + entrenamiento |
| **B. Solo color** | Objetos grandes con color muy distinto; YOLO falla o no aplica; el color HSV hace casi todo. | Rápido pero frágil con piezas pequeñas o grises |

---

## 2. Por qué las resistencias son especialmente difíciles

- Ocupan **pocos píxeles** a 640×640 si la cámara está lejos.
- Las **bandas de color** mezclan varios tonos en una caja pequeña: el “color dominante” puede **no** ser el de la primera banda.
- Ordenar por **valor óhmico** (leyendo bandas) es otro problema: haría falta **OCR o un modelo específico**, no está en este repositorio.

**Recomendación práctica:** coloca la cámara **más cerca**, buena luz difusa, fondo **mate** (gris/blanco) y, si puedes, **bandejas separadas** por tipo de pieza en lugar de confiar solo en el color de la resistencia.

---

## 3. Cautín y herramientas medianas

Un **cautín** o **multímetro** es más grande que una resistencia: un modelo custom con decenas de fotos por clase suele **funcionar razonablemente** si:

- Fondo fijo o similar al que usarás en producción.
- Iluminación estable (evita sombras duras).
- Clases con nombres **claros** en el dataset (ej. `cautin`, `multimetro`, `pinza`).

---

## 4. Cómo enlazar tus clases con los recipientes del brazo

1. Entrena YOLO (Ultralytics, Roboflow, etc.) y exporta el modelo siguiendo la misma idea que `perception/vision/detection/models/yolo11s_ncnn_model/` o apunta `model_loader.py` a tu `.pt` / NCNN.
2. Anota los **nombres exactos** de las clases que imprime el modelo (sensibles a mayúsculas según exportación; en COCO-style suelen ser strings fijos).
3. En `arm_system/config_sistema.py`, rellena **`YOLO_LAB_CLASE_A_COLOR`**: cada clave es el nombre de clase del modelo, cada valor es el color del recipiente (`rojo`, `azul`, `verde`, `amarillo`, `naranja`, `morado`).

El detector fusiona el mapa de laboratorio con el mapa por defecto de objetos “domésticos” en `color_detector.py`.

---

## 5. Ajustes técnicos que suelen ayudar

- **`CONFIANZA_MINIMA_DETECCION`** en `config_sistema.py`: si hay falsos positivos, súbela un poco; si no detecta nada, bájala con cuidado.
- **Resolución de inferencia** (`imgsz` en `autonomous_brain.py` en la llamada a `predict`): subir a **768 o 960** puede ayudar a piezas pequeñas a costa de **más CPU/tiempo** en la Pi.
- **Recipientes por color** en la imagen: sigue siendo HSV; calibra con el botón **Calibrar color** en la web.

---

## 6. Resumen

- **No**, el proyecto “tal cual” con COCO **no** está pensado para resistencias y cautín.
- **Sí**, la arquitectura (detectar caja → color / clase → recipiente) **sirve** para laboratorio si aportas **modelo + mapeo** en `YOLO_LAB_CLASE_A_COLOR` y un **montaje físico** razonable.

Si más adelante quieres, se puede añadir un modo “solo laboratorio” que ignore clases COCO que no te interesen o un segundo pipeline para piezas muy pequeñas (segmentación / plantillas); eso ya es una iteración nueva.

---

## 7. Checklist general del proyecto

Laboratorio es solo una parte. Para no saltarte calibración, seguridad ni audio, usa **[PUESTA_EN_MARCHA.md](PUESTA_EN_MARCHA.md)**.

Recipientes, luz y modo sin simulación: **[RECIPIENTES_Y_LUZ.md](RECIPIENTES_Y_LUZ.md)**.
