# Guía de presentación técnica

Este documento está diseñado para ayudarte a presentar el proyecto de forma honesta, profesional y competitiva ante jurados técnicos.

## Mensaje central

Este proyecto es una **plataforma robótica modular con visión artificial y automatización física asistida** sobre hardware real.

El foco está en integrar correctamente:

- percepción visual básica funcional,
- lógica basada en reglas para clasificación y manipulación,
- control seguro de servomotores reales,
- una interfaz de supervisión web.

## Qué puedes decir con seguridad

- el proyecto integra hardware real y software en Raspberry Pi,
- controla servos a través de PCA9685,
- captura imagen con cámara CSI,
- usa un modelo YOLO preentrenado para detección de objetos,
- usa análisis HSV para clasificación de color,
- ejecuta movimientos secuenciales predefinidos,
- incluye una interfaz Flask para supervisión,
- incorpora seguridad de hardware mediante `SafeController` y `HW_LOCK`.

## Qué no debes decir

- no digas que el sistema es una IA autónoma avanzada,
- no digas que aprende por sí solo,
- no digas que tiene aprendizaje adaptativo,
- no digas que implementa SLAM o RL,
- no digas que el modelo está entrenado desde cero con un dataset propio robusto,
- no digas que la autonomía es total.

## Terminología adecuada

Usa estas expresiones:

- automatización asistida
- percepción visual
- lógica basada en reglas
- integración hardware/software
- control físico seguro
- arquitectura modular
- plataforma embebida
- supervisión web
- robustez de software

## Estructura de presentación recomendada

1. Introducción breve
   - Propósito del proyecto.
   - Hardware y software principales.

2. Alcance real del sistema
   - Qué hace.
   - Qué no hace.

3. Arquitectura modular
   - percepción
   - decisión / automatización asistida
   - actuación / control
   - interfaz y supervisión

4. Seguridad y robustez
   - `SafeController`
   - `HW_LOCK`
   - límites de ángulo y validación

5. Demostración práctica
   - iniciar interfaz web,
   - mostrar vídeo de la cámara,
   - iniciar ciclo asistido,
   - mostrar control manual y emergencia.

6. Conclusión técnica
   - su verdadero valor está en la integración,
   - está preparado para mejoras futuras,
   - es un prototipo realista y defendible.

## Estructura técnica del proyecto

### Percepción

- `CameraManager` captura imagen de la cámara CSI.
- `DetectionModel` carga un modelo YOLO preentrenado.
- `DetectorColor` clasifica el color dominante con HSV.

### Decisión / Automatización asistida

- `CerebroAutonomo` orquesta el ciclo de escaneo, detección y ejecución.
- El sistema aplica reglas y secuencias predefinidas.
- Acepta reintentos y fallback ante fallos de visión.

### Actuación

- `ArmController` controla servos con ángulos y pulsos PWM.
- `ControladorServo` agrega compatibilidad legacy y movimiento por tiempo.

### Supervisión

- `autonomous_web.py` expone un panel de control y diagnóstico.
- Permite ver estado, iniciar ciclos y manejar calibración.

## Seguridad mecánica

### `SafeController`

Describe cómo protege el hardware:

- valida ángulos antes de mover,
- aplica límites seguros por articulación,
- evita saltos peligrosos,
- limita la frecuencia de comandos,
- interpola movimientos en pasos suaves,
- ofrece emergency stop y reset.

### `HW_LOCK`

Explica por qué es importante:

- es un bloqueo global para el bus PCA9685,
- evita que dos subsistemas escriban al mismo tiempo,
- protege el hardware frente a accesos concurrentes.

## Puntos fuertes para jurado

- integración real de hardware y software,
- visión artificial aplicada en un sistema embebido,
- control de servos con seguridad,
- arquitectura modular y extensible,
- foco en robustez y funcionamiento físico.

## Preguntas del jurado y respuestas sugeridas

### ¿El sistema aprende solo?

No. Actualmente utiliza detección visual con un modelo preentrenado y toma decisiones basadas en reglas. El aprendizaje automático avanzado no está implementado en este repositorio.

### ¿Qué tipo de autonomía tiene?

Autonomía asistida: el sistema puede ejecutar una secuencia de tareas con supervisión, pero no es una autonomía inteligente compleja.

### ¿Por qué usaste YOLO?

Porque permite detección de objetos en tiempo real con un enfoque probado. Se utiliza un modelo preentrenado para acelerar la integración y concentrarse en el control físico.

### ¿Qué pasaría si el modelo no reconoce el objeto?

Si el modelo no reconoce un objeto, la lógica de visión puede fallar en esa instancia. Por eso el sistema requiere ajustes y calibración para el entorno real.

### ¿Qué mejoras propones?

- entrenar un modelo propio con datos reales,
- agregar sensores de contacto o fuerza,
- refinar la planificación de trayectoria,
- mejorar la detección de colores y condiciones de iluminación.

## Demo técnica sugerida

1. Mostrar el hardware real y explicar componentes.
2. Iniciar la interfaz web.
3. Mostrar el vídeo en vivo y las detecciones.
4. Activar el modo asistido y observar la secuencia de movimientos.
5. Mostrar cómo funciona el control manual y el emergency stop.
6. Explicar limitaciones y cómo se puede evolucionar.

## Mensaje final para el jurado

> Este proyecto no es una IA industrial completa; es una plataforma embebida realista que integra visión artificial y control de hardware con seguridad. Su mérito está en hacer funcionar un brazo físico con percepción visual y lógica de automatización asistida.
