# CONFIGURACIÓN DEL SISTEMA - Brazo Robótico
SERVOS_HABILITADOS = True
# Base horizontal: servo MG996R ~180° en PCA9685 canal 4 (servo_config_legacy.json → "base").
# Pon True solo si usas motor paso a paso + TMC2208 en GPIO 17/18/19 en lugar del servo de base.
STEPPER_HABILITADO = False
CAMARA_HABILITADA = True

# Servos: Hombro(0), Codo(1), Muñeca(2), Pinza(3), Base(4)

MODO_AUTONOMO = True
COLORES_RECIPIENTES = {
    'rojo': {'h_min': 0, 'h_max': 10, 's_min': 100, 'v_min': 100},
    'azul': {'h_min': 100, 'h_max': 130, 's_min': 100, 'v_min': 100},
    'verde': {'h_min': 40, 'h_max': 80, 's_min': 100, 'v_min': 100},
    'amarillo': {'h_min': 20, 'h_max': 35, 's_min': 100, 'v_min': 100},
}
CONFIANZA_MINIMA_DETECCION = 0.45
MAX_REINTENTOS_AGARRE = 3
VELOCIDAD_AUTONOMA = 0.4

# Si False: nunca se usan objetos/recipientes ficticios cuando falla YOLO, cámara o color.
# El autónomo no moverá el brazo “a ciegas” por datos simulados. Pon True solo en PC sin hardware.
PERMITIR_DETECCION_SIMULADA = False

# Voz (micrófono + TTS). Requiere: pip install -r ../requirements-voice.txt
# y en Pi: sudo apt install espeak-ng portaudio19-dev
VOZ_HABILITADA = False
VOZ_IDIOMA_RECONOCIMIENTO = "es-ES"  # Google Speech Recognition
VOZ_ANUNCIAR_EVENTOS = True  # mensajes breves al iniciar/detener desde voz o API
# Índice PyAudio del micrófono (None = predeterminado del sistema). Ver HARDWARE_AUDIO.md
VOZ_MIC_DEVICE_INDEX = None

# --- Laboratorio / mesa de trabajo (YOLO propio) ---
# El modelo COCO del proyecto NO conoce resistencias, cautín, etc. Entrena un YOLO con
# tus clases y pon aquí el mismo nombre de clase que exporta el modelo -> color de recipiente.
# Claves válidas de color: rojo, azul, verde, amarillo, naranja, morado (como en color_detector).
# Ver LAB_WORKBENCH.md
YOLO_LAB_CLASE_A_COLOR = {
    # Ejemplos (descomenta y ajusta a los nombres EXACTOS de tu dataset):
    # 'cautin': 'rojo',
    # 'cautín': 'rojo',
    # 'resistencia': 'amarillo',
    # 'resistor': 'amarillo',
    # 'condensador': 'azul',
    # 'capacitor': 'azul',
    # 'pinza': 'verde',
    # 'multimetro': 'naranja',
}
