# Guía de contribución

Gracias por interesarte en mejorar **BrazoRoboticoConIA**. Este documento resume cómo proponer cambios y reportar problemas de forma ordenada.

## Código de conducta

Participar en este proyecto implica aceptar el [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md). Sé respetuoso y constructivo.

## Cómo reportar errores

1. **Busca** en issues existentes por si ya está reportado.
2. Abre un **issue** con:
   - Descripción del comportamiento esperado vs. observado.
   - Entorno: Raspberry Pi (modelo), sistema operativo, rama/commit del repo.
   - Pasos para reproducir y, si aplica, logs o capturas (sin datos sensibles).

Para **vulnerabilidades de seguridad**, no uses el issue tracker público: lee [SECURITY.md](SECURITY.md).

## Cómo proponer cambios

1. **Fork** del repositorio y rama nueva desde `main` (`feature/…` o `fix/…`).
2. Cambios **acotados**: un tema por pull request facilita la revisión.
3. **Prueba** en la medida de lo posible:
   - En PC: imports y tests que no requieran hardware.
   - En Raspberry Pi: flujo web, cámara y servos según lo que toques.
4. **Commits** con mensajes claros en español o inglés (mantén coherencia con el historial del repo).
5. Abre un **Pull Request** describiendo qué hace el cambio y por qué.

## Estilo de código

- Sigue el estilo ya presente en los archivos que modifiques (imports, logging, nombres).
- Evita cambios masivos de formato no relacionados con la corrección o la función nueva.
- Si añades dependencias, justifícalas en el PR y actualiza `requirements.txt` o `requirements-voice.txt` según corresponda.

## Documentación

- Si el cambio afecta al uso del brazo, la instalación o la seguridad, actualiza el `.md` relevante (`README.md`, `PUESTA_EN_MARCHA.md`, `REFERENCE.md`, etc.) en el mismo PR cuando tenga sentido.

## Hardware

Muchas contribuciones requieren **Raspberry Pi**, PCA9685, cámara, etc. Si no puedes probar en hardware real, indícalo en el PR para que otro colaborador pueda validar.
