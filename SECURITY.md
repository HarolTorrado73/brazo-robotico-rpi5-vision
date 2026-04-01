# Política de seguridad

## Versiones soportadas

Este repositorio es un proyecto de integración hardware/software en evolución. Las correcciones de seguridad se aplican preferentemente en la rama principal (`main`). Si mantienes un fork o una rama larga, fusiona o reaplica los parches con regularidad.

## Cómo reportar una vulnerabilidad

**No abras un issue público** para informar de vulnerabilidades que puedan facilitar abusos (credenciales, RCE, exposición de datos, etc.).

1. **Contacto privado:** envía un correo al mantenedor del repositorio con el asunto `[SECURITY]` y una descripción clara del problema.
2. Incluye pasos para reproducir el fallo, versión del software/commit y, si aplica, impacto estimado.
3. Intentaremos **confirmar recepción en un plazo razonable** y coordinar una solución antes de divulgar detalles públicamente.

Si no hay dirección de correo publicada en el perfil de GitHub del propietario del repo, usa la función **“Security” → “Report a vulnerability”** de GitHub si está habilitada para este proyecto.

## Alcance

- **Fuera de alcance habitual:** problemas que requieran acceso físico no autorizado al robot, redes aisladas de laboratorio o configuraciones deliberadamente inseguras documentadas como tales.
- **Dependencias de terceros:** muchas alertas provienen de herramientas como `pip audit` o dependabot; conviene actualizar `requirements.txt` y probar en la Raspberry Pi antes de desplegar.

## Buenas prácticas para quien despliega el sistema

- No subas **claves API**, tokens de voz en la nube ni archivos `.env` al repositorio (están ignorados en `.gitignore`).
- Restringe el acceso a la interfaz web (red local, firewall) si expones el puerto del Flask.
- Mantén el sistema operativo de la Pi y las dependencias Python actualizadas cuando sea posible.
