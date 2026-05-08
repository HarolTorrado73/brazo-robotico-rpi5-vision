# Guía de contribución

Gracias por interesarte en mejorar **BrazoRoboticoConIA**. Este documento resume cómo proponer cambios y reportar problemas de forma ordenada.

## Código de conducta

Participar en este proyecto implica aceptar el [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md). Sé respetuoso, claro y constructivo.

## Cómo reportar errores

1. **Busca** en issues existentes antes de crear uno nuevo.
2. Abre un **issue** con:
   - Descripción del comportamiento esperado vs. observado.
   - Entorno: Raspberry Pi (modelo), sistema operativo, rama/commit del repo.
   - Pasos para reproducir el problema.
   - Logs, capturas o terminales relevantes (sin datos sensibles).
3. Si el problema es de seguridad, no uses issues públicos: lee [SECURITY.md](SECURITY.md).

## Cómo contribuir desde un fork

1. Haz **fork** del repositorio en GitHub.
2. Clona tu fork localmente:
   ```bash
   git clone https://github.com/<tu_usuario>/BrazoRoboticoConIA-v2.git
   cd BrazoRoboticoConIA-v2
   ```
3. Crea una rama nueva basada en `main`:
   ```bash
   git checkout main
   git pull origin main
   git checkout -b feature/<descripcion-corta>
   ```
4. Mantén los cambios **pequeños y enfocados**: un PR por función o corrección.
5. No hagas push directo a `main`; usa siempre tu rama de trabajo.
6. Antes de abrir el PR, sincroniza tu rama con `main` para evitar conflictos.

## Pruebas y validación

- Ejecuta las pruebas o comandos pertinentes antes de enviar cambios.
- Si no tienes hardware real, indica claramente en el PR qué partes no pudiste verificar.
- Comprueba:
  - `pip install -r requirements.txt` y/o `requirements-voice.txt`
  - que no haya errores de sintaxis en Python
  - que los cambios no rompan la documentación o la estructura del proyecto

## Commits y mensajes

- Usa mensajes breves, descriptivos y en el mismo idioma del repositorio.
- Ejemplo: `Mejora: guía de contribución para forks` o `Fix: streaming de cámara con rpicam-still`.
- Evita commits con cambios no relacionados a la misma tarea.

## Cómo crear un Pull Request

1. Empuja tu rama al fork:
   ```bash
   git push origin feature/<descripcion-corta>
   ```
2. Abre un pull request contra `main` en el repositorio original.
3. Describe:
   - el objetivo del cambio
   - los archivos principales modificados
   - qué se probó y en qué entorno
   - si quedan aspectos pendientes o riesgos
4. Si el PR resuelve un issue, menciona `Closes #<número>`.

## Estilo de código

- Sigue el estilo ya presente en los archivos que modifiques (imports, logging, nombres).
- Mantén la indentación y formato coherentes.
- Evita cambios masivos de formato cuando no sean estrictamente necesarios.
- Si añades dependencias, justifica su uso y actualiza `requirements.txt` o `requirements-voice.txt`.

## Documentación

- Si el cambio afecta al uso del brazo, la instalación, la configuración o la seguridad, actualiza el `.md` relevante.
- Ejemplos: `README.md`, `PUESTA_EN_MARCHA.md`, `REFERENCE.md`, `HARDWARE_AUDIO.md`.

## Hardware y validación en Raspberry Pi

- Muchas contribuciones dependen de hardware real: cámara CSI, PCA9685, servos y alimentación.
- Si no puedes probar todos los cambios en hardware, indícalo en el PR y describe qué sí verificaste.

## Comunicación

- Si no estás seguro de un cambio grande, abre un issue antes de trabajar.
- Describe claramente el problema y las ideas de solución.
- Usa el issue tracker para coordinar cambios mayores y evitar trabajo duplicado.

