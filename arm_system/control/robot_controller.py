import time
import threading
import logging as log
import json
import os

try:
    from arm_system import hw_bus as _hw_bus
except ImportError:
    try:
        import hw_bus as _hw_bus  # type: ignore
    except ImportError:
        _hw_bus = None  # type: ignore


def _blinka_pca9685():
    """Import diferido: evita ModuleNotFoundError al cargar el módulo sin Blinka instalada."""
    import board  # noqa: PLC0415
    import busio  # noqa: PLC0415
    from adafruit_pca9685 import PCA9685  # noqa: PLC0415

    return board, busio, PCA9685


class ControladorServo:
    """PCA9685: servos posicionales (~180°) por defecto en v2, o continuos si tipo_servo=continuo."""

    # Límite superior de espera tras enviar pulso (el servo suele asentar antes).
    TIEMPO_ASENTAMIENTO_MAX = 1.35

    def __init__(self, direccion_i2c=0x40, frecuencia=50):
        """Inicializar controlador PCA9685"""
        # Para Raspberry Pi 5: usar GPIO 3 (SCL) y GPIO 2 (SDA) - puerto I2C1
        # Estos son los pines físicos 5 y 3 respectivamente
        try:
            board_mod, busio_mod, PCA9685_cls = _blinka_pca9685()
            self.i2c = busio_mod.I2C(board_mod.D3, board_mod.D2)
            log.info("I2C inicializado en GPIO3/GPIO2 (bus I2C1)")
        except ImportError as e:
            msg = (
                "Falta Adafruit Blinka (módulo 'board'). En el venv: "
                "pip install adafruit-blinka adafruit-circuitpython-pca9685"
            )
            log.error("%s — %s", msg, e)
            raise ImportError(msg) from e
        except Exception as e:
            log.error(f"Error inicializando I2C en GPIO3/GPIO2: {e}")
            log.error("Verifica que I2C esté habilitado en raspi-config")
            raise

        self.pca = PCA9685_cls(self.i2c, address=direccion_i2c)
        self.pca.frequency = frecuencia
        self.servos = {}
        
        # Cargar pulsos neutrales calibrados desde servo_config_legacy.json
        self.pulsos_neutrales = self._cargar_pulsos_neutrales()
        log.info(f"Pulsos neutrales cargados: {self.pulsos_neutrales}")
        
        # Diagnóstico y seguridad
        # Si es True, se aplicará un pequeño pulso de "hold" en lugar del pulso
        # neutral exacto cuando termine el movimiento.
        # Por defecto: False (mantener comportamiento existente)
        self.hold_after_move = False
        # Cantidad (en microsegundos) para desplazar del neutral cuando está en hold.
        # Usar valores pequeños (ej. 50-200) durante pruebas. Esto no convierte
        # servos continuos en posicionales - es solo un pequeño bias/pulso.
        self.hold_pulse_offset = 100

    def _cargar_pulsos_neutrales(self):
        """Cargar configuracion completa de pulsos desde servo_config_legacy.json"""
        # Formato por articulación (pulso/tiempos) para la web y ControladorRobotico.
        # El archivo principal servo_config.json queda reservado para ArmController (grados + joints).
        config_path = os.path.join(os.path.dirname(__file__), '..', 'servo_config_legacy.json')

        # Valores por defecto: servos estándar ~180° (MG996R/SG90 en modo angular, etc.)
        default_config = {
            'shoulder': {
                'tipo_servo': 'posicional_180', 'pulso_min': 800, 'pulso_max': 2200,
                'pulso_neutral': 1500, 'pulso_hold': 1500,
                'rango_pulso': 400, 'velocidad_min': 0.2,
                'tiempo_max_positivo': 2.8, 'tiempo_max_negativo': 2.8,
            },
            'elbow': {
                'tipo_servo': 'posicional_180', 'pulso_min': 850, 'pulso_max': 2150,
                'pulso_neutral': 1500, 'pulso_hold': 1500,
                'rango_pulso': 400, 'velocidad_min': 0.2,
                'invertido': True,
                'tiempo_max_positivo': 2.6, 'tiempo_max_negativo': 2.6,
            },
            'wrist': {
                'tipo_servo': 'posicional_180', 'pulso_min': 900, 'pulso_max': 2100,
                'pulso_neutral': 1500, 'pulso_hold': 1500,
                'rango_pulso': 350, 'velocidad_min': 0.15,
                'invertido': True,
                'tiempo_max_positivo': 2.0, 'tiempo_max_negativo': 2.0,
            },
            'gripper': {
                'tipo_servo': 'posicional_180', 'pulso_min': 1000, 'pulso_max': 2200,
                'pulso_neutral': 1600, 'pulso_hold': 1600,
                'pulso_abrir': 2200, 'pulso_cerrar': 1000,
                'rango_pulso': 400, 'velocidad_min': 0.2,
                'tiempo_max_positivo': 1.2, 'tiempo_max_negativo': 1.2,
            },
            'base': {
                'tipo_servo': 'posicional_180', 'pulso_min': 800, 'pulso_max': 2200,
                'pulso_neutral': 1500, 'pulso_hold': 1500,
                'rango_pulso': 400, 'velocidad_min': 0.2,
                'tiempo_max_positivo': 2.5, 'tiempo_max_negativo': 2.5,
            },
        }

        self._config_path = config_path

        try:
            if os.path.exists(config_path):
                with open(config_path, 'r') as f:
                    config = json.load(f)
                    pulsos = {}
                    for nombre, datos in config.items():
                        # Ignora metadatos/globales que no son bloques de articulación
                        # (ej: "_comment": "...") para evitar errores tipo "'str' has no attribute 'get'".
                        if not isinstance(datos, dict):
                            continue
                        dfl = default_config.get(nombre, {})
                        t_pos_keys = ['tiempo_max_arriba', 'tiempo_max_extender',
                                      'tiempo_max_horario', 'tiempo_max_abrir',
                                      'tiempo_max_positivo']
                        t_neg_keys = ['tiempo_max_abajo', 'tiempo_max_contraer',
                                      'tiempo_max_antihorario', 'tiempo_max_cerrar',
                                      'tiempo_max_negativo']

                        t_pos = dfl.get('tiempo_max_positivo', 5.0)
                        for k in t_pos_keys:
                            if k in datos:
                                t_pos = datos[k]
                                break

                        t_neg = dfl.get('tiempo_max_negativo', 5.0)
                        for k in t_neg_keys:
                            if k in datos:
                                t_neg = datos[k]
                                break

                        tipo = datos.get('tipo_servo', dfl.get('tipo_servo', 'posicional_180'))
                        pmin = datos.get('pulso_min', dfl.get('pulso_min', 800))
                        pmax = datos.get('pulso_max', dfl.get('pulso_max', 2200))
                        pulsos[nombre] = {
                            'tipo_servo': tipo,
                            'pulso_min': pmin,
                            'pulso_max': pmax,
                            'neutral': datos.get('pulso_neutral', dfl.get('pulso_neutral', 1500)),
                            'hold': datos.get('pulso_hold', dfl.get('pulso_hold', 1500)),
                            'rango_pulso': datos.get('rango_pulso', dfl.get('rango_pulso', 500)),
                            'velocidad_min': datos.get('velocidad_min', dfl.get('velocidad_min', 0.5)),
                            'pulso_abrir': datos.get('pulso_abrir', dfl.get('pulso_abrir', 0)),
                            'pulso_cerrar': datos.get('pulso_cerrar', dfl.get('pulso_cerrar', 0)),
                            'invertido': datos.get('invertido', dfl.get('invertido', False)),
                            'tiempo_max_positivo': t_pos,
                            'tiempo_max_negativo': t_neg,
                        }
                    log.info(f"Configuracion cargada desde {config_path}")
                    return pulsos
            else:
                log.warning(f"No se encontro {config_path}, usando valores por defecto")
        except Exception as e:
            log.error(f"Error cargando servo_config_legacy.json: {e}")

        return {k: {
            'tipo_servo': v.get('tipo_servo', 'posicional_180'),
            'pulso_min': v.get('pulso_min', 800),
            'pulso_max': v.get('pulso_max', 2200),
            'neutral': v['pulso_neutral'], 'hold': v['pulso_hold'],
            'rango_pulso': v.get('rango_pulso', 500),
            'velocidad_min': v.get('velocidad_min', 0.5),
            'pulso_abrir': v.get('pulso_abrir', 0),
            'pulso_cerrar': v.get('pulso_cerrar', 0),
            'invertido': v.get('invertido', False),
            'tiempo_max_positivo': v.get('tiempo_max_positivo', 5.0),
            'tiempo_max_negativo': v.get('tiempo_max_negativo', 5.0),
        } for k, v in default_config.items()}

    @staticmethod
    def _es_servo_posicional(servo):
        t = (servo or {}).get('tipo_servo', 'posicional_180')
        # Acepta etiquetas explícitas de rango angular (180/270) además de aliases clásicos.
        return str(t).lower() in (
            'posicional',
            'posicional_180',
            'posicional_270',
            'standard',
            'angular',
        )

    def agregar_servo(self, nombre, canal, pulso_min=None, pulso_max=None):
        """Registra un servo. Los limites en us suelen venir de servo_config (pulso_min/max)."""
        config = self.pulsos_neutrales.get(nombre, {
            'tipo_servo': 'posicional_180', 'pulso_min': 800, 'pulso_max': 2200,
            'neutral': 1500, 'hold': 1500, 'rango_pulso': 500,
            'velocidad_min': 0.5, 'pulso_abrir': 0, 'pulso_cerrar': 0,
            'invertido': False,
            'tiempo_max_positivo': 5.0, 'tiempo_max_negativo': 5.0,
        })
        pmin = pulso_min if pulso_min is not None else config.get('pulso_min', 800)
        pmax = pulso_max if pulso_max is not None else config.get('pulso_max', 2200)
        self.servos[nombre] = {
            'nombre': nombre,
            'canal': canal,
            'tipo_servo': config.get('tipo_servo', 'posicional_180'),
            'pulso_min': pmin,
            'pulso_max': pmax,
            'pulso_neutral': config['neutral'],
            'pulso_hold': config['hold'],
            'rango_pulso': config.get('rango_pulso', 500),
            'velocidad_min': config.get('velocidad_min', 0.5),
            'pulso_abrir': config.get('pulso_abrir', 0),
            'pulso_cerrar': config.get('pulso_cerrar', 0),
            'invertido': config.get('invertido', False),
            'tiempo_max_positivo': config.get('tiempo_max_positivo', 5.0),
            'tiempo_max_negativo': config.get('tiempo_max_negativo', 5.0),
            'posicion_estimada': 0.5,
            'ultimo_pulso': config['neutral'],
        }
        modo = 'posicional' if self._es_servo_posicional(self.servos[nombre]) else 'continuo'
        log.info(f"Servo '{nombre}' ({modo}): canal={canal} rango_us=[{pmin},{pmax}] "
                 f"t_max=[+{config.get('tiempo_max_positivo', 5.0):.1f}s "
                 f"-{config.get('tiempo_max_negativo', 5.0):.1f}s]")

    def _us_a_duty(self, pulso_us):
        """Convierte microsegundos de pulso a duty_cycle (0-0xFFFF) para 50Hz."""
        return int(pulso_us / 20000 * 0xFFFF)

    def _pulso_desde_posicion(self, servo, pos_norm):
        """pos_norm en [0,1] -> ancho de pulso en us (pinza interpola cerrar..abrir)."""
        pos_norm = max(0.0, min(1.0, float(pos_norm)))
        nombre = servo.get('nombre', '')
        inv = servo.get('invertido', False)
        if nombre == 'gripper' and servo.get('pulso_abrir') and servo.get('pulso_cerrar'):
            pa = int(servo['pulso_abrir'])
            pc = int(servo['pulso_cerrar'])
            lo, hi = (pc, pa) if pc <= pa else (pa, pc)
            pn = 1.0 - pos_norm if inv else pos_norm
            p = lo + pn * (hi - lo)
            return int(max(lo, min(hi, round(p))))
        p_min = int(servo['pulso_min'])
        p_max = int(servo['pulso_max'])
        if p_min > p_max:
            p_min, p_max = p_max, p_min
        p = p_min + pos_norm * (p_max - p_min)
        if inv:
            p = p_max + p_min - p
        return int(max(p_min, min(p_max, round(p))))

    def _safe_set_channel_duty(self, channel, duty_cycle, retries=3, retry_delay=0.05):
        """Escribe duty_cycle al PCA9685 con reintentos en caso de errores I2C transitorios."""
        for intento in range(1, retries + 1):
            try:
                self.pca.channels[channel].duty_cycle = duty_cycle
                return
            except OSError as e:
                log.warning(
                    f"[Servo] Error I2C canal={channel}, duty={duty_cycle}: {e} "
                    f"(intento {intento}/{retries})"
                )
                if intento == retries:
                    raise
                time.sleep(retry_delay)

    def aplicar_pulso(self, nombre, pulso_us):
        """Aplica un pulso directo en microsegundos a un servo."""
        if nombre not in self.servos:
            return
        servo = self.servos[nombre]
        pulso_us = max(servo['pulso_min'], min(servo['pulso_max'], pulso_us))
        self._safe_set_channel_duty(servo['canal'], self._us_a_duty(pulso_us))

    def mover_por_tiempo(self, nombre, direccion, tiempo_segundos, velocidad=0.5):
        """Mueve un servo: posicional (~180°) interpola ángulo en [0,1]; continuo mantiene lógica legacy.

        Direccion +1 = subir / abrir / positivo; -1 = bajar / cerrar; 0 = mantener posición actual.

        Adquiere HW_LOCK durante todo el comando (incluyendo tiempo de asentamiento) para
        garantizar exclusión mutua con SafeController. Si el lock no está disponible en
        300 ms, el comando se descarta con un aviso.
        """
        if nombre not in self.servos:
            log.error(f"Servo {nombre} no configurado")
            return

        if _hw_bus is not None:
            if not _hw_bus.HW_LOCK.acquire(timeout=0.30):
                log.warning(
                    f"[Servo] mover_por_tiempo('{nombre}'): HW_LOCK no disponible "
                    f"(SafeController activo). Comando descartado."
                )
                return
            try:
                servo = self.servos[nombre]
                if self._es_servo_posicional(servo):
                    self._mover_por_tiempo_posicional(nombre, servo, direccion, tiempo_segundos, velocidad)
                else:
                    self._mover_por_tiempo_continuo(nombre, servo, direccion, tiempo_segundos, velocidad)
            finally:
                _hw_bus.HW_LOCK.release()
        else:
            servo = self.servos[nombre]
            if self._es_servo_posicional(servo):
                self._mover_por_tiempo_posicional(nombre, servo, direccion, tiempo_segundos, velocidad)
            else:
                self._mover_por_tiempo_continuo(nombre, servo, direccion, tiempo_segundos, velocidad)

    def _mover_por_tiempo_posicional(self, nombre, servo, direccion, tiempo_segundos, velocidad):
        if direccion == 0:
            pulso = self._pulso_desde_posicion(servo, servo['posicion_estimada'])
            self.pca.channels[servo['canal']].duty_cycle = self._us_a_duty(pulso)
            servo['ultimo_pulso'] = pulso
            return

        t_max_dir = (servo['tiempo_max_positivo'] if direccion > 0
                     else servo['tiempo_max_negativo'])
        pos = servo['posicion_estimada']

        if direccion > 0:
            margen = (1.0 - pos) * t_max_dir
        else:
            margen = pos * t_max_dir

        if margen <= 0:
            log.warning(f"[Servo] {nombre}: limite (pos={pos:.2f}, dir={direccion})")
            return

        tiempo_original = tiempo_segundos
        tiempo_segundos = min(tiempo_segundos, margen)
        if tiempo_segundos < tiempo_original:
            log.info(f"[Servo] {nombre}: tiempo limitado {tiempo_original:.2f}s -> "
                     f"{tiempo_segundos:.2f}s (margen={margen:.2f}s)")

        if t_max_dir <= 0:
            delta = 0.0
        else:
            delta = tiempo_segundos / t_max_dir

        if direccion > 0:
            servo['posicion_estimada'] = min(1.0, pos + delta)
        else:
            servo['posicion_estimada'] = max(0.0, pos - delta)

        pulso = self._pulso_desde_posicion(servo, servo['posicion_estimada'])
        inv_tag = " [INV]" if servo.get('invertido') else ""
        log.info(f"[Servo] {nombre}{inv_tag} (180°): dir={direccion} t_cmd={tiempo_segundos:.2f}s "
                 f"pulso={pulso}us pos={servo['posicion_estimada']:.2f}")
        self._safe_set_channel_duty(servo['canal'], self._us_a_duty(pulso))
        servo['ultimo_pulso'] = pulso

        asent = max(0.06, min(float(tiempo_segundos), self.TIEMPO_ASENTAMIENTO_MAX))
        time.sleep(asent)
        self._safe_set_channel_duty(servo['canal'], self._us_a_duty(pulso))
        log.info(f"[Servo] {nombre}: manteniendo {pulso}us (pos={servo['posicion_estimada']:.2f})")

    def _mover_por_tiempo_continuo(self, nombre, servo, direccion, tiempo_segundos, velocidad):
        """Comportamiento original para tipo_servo=continuo."""
        pulso_neutral = servo['pulso_neutral']
        rango = servo['rango_pulso']
        vel_min = servo['velocidad_min']
        invertido = servo.get('invertido', False)
        vel_efectiva = max(velocidad, vel_min)

        if direccion != 0:
            t_max_dir = (servo['tiempo_max_positivo'] if direccion > 0
                         else servo['tiempo_max_negativo'])
            pos = servo['posicion_estimada']

            if direccion > 0:
                margen = (1.0 - pos) * t_max_dir
            else:
                margen = pos * t_max_dir

            if margen <= 0:
                log.warning(f"[Servo] {nombre}: limite fisico alcanzado "
                            f"(pos={pos:.2f}, dir={direccion})")
                return

            tiempo_original = tiempo_segundos
            tiempo_segundos = min(tiempo_segundos, margen)
            if tiempo_segundos < tiempo_original:
                log.info(f"[Servo] {nombre}: tiempo limitado {tiempo_original:.2f}s -> "
                         f"{tiempo_segundos:.2f}s (margen={margen:.2f}s)")

        if nombre == 'gripper' and servo['pulso_abrir'] and servo['pulso_cerrar']:
            if direccion == 1:
                pulso = servo['pulso_abrir']
            elif direccion == -1:
                pulso = servo['pulso_cerrar']
            else:
                pulso = pulso_neutral
        else:
            if direccion == 0:
                pulso = pulso_neutral
            else:
                desplazamiento = rango * vel_efectiva
                if invertido:
                    if direccion == 1:
                        pulso = pulso_neutral + desplazamiento
                    else:
                        pulso = pulso_neutral - desplazamiento
                else:
                    if direccion == 1:
                        pulso = pulso_neutral - desplazamiento
                    else:
                        pulso = pulso_neutral + desplazamiento

        pulso = max(servo['pulso_min'], min(servo['pulso_max'], pulso))

        inv_tag = " [INV]" if invertido else ""
        log.info(f"[Servo] {nombre}{inv_tag}: dir={direccion} t={tiempo_segundos:.2f}s "
                 f"pulso={pulso:.0f}us vel={vel_efectiva:.2f} pos={servo['posicion_estimada']:.2f}")
        self._safe_set_channel_duty(servo['canal'], self._us_a_duty(pulso))

        time.sleep(tiempo_segundos)

        if direccion != 0:
            t_max_dir = (servo['tiempo_max_positivo'] if direccion > 0
                         else servo['tiempo_max_negativo'])
            if t_max_dir > 0:
                delta = tiempo_segundos / t_max_dir
                if direccion > 0:
                    servo['posicion_estimada'] = min(1.0, servo['posicion_estimada'] + delta)
                else:
                    servo['posicion_estimada'] = max(0.0, servo['posicion_estimada'] - delta)

        pulso_hold = servo['pulso_hold']
        self._safe_set_channel_duty(servo['canal'], self._us_a_duty(pulso_hold))
        servo['ultimo_pulso'] = pulso_hold
        log.info(f"[Servo] {nombre}: STOP -> {pulso_hold}us "
                 f"(pos={servo['posicion_estimada']:.2f})")

    def obtener_posiciones_estimadas(self):
        """Retorna un dict con la posicion estimada (0.0-1.0) de cada servo."""
        return {nombre: servo['posicion_estimada'] for nombre, servo in self.servos.items()}

    def resetear_posiciones(self):
        """Resetear todas las posiciones estimadas a 0.5 (centro/home)."""
        for servo in self.servos.values():
            servo['posicion_estimada'] = 0.5

    def iniciar_refresco_anti_drift(self, intervalo=2.0):
        """Hilo que reenvía el pulso actual (180°: mantiene ángulo; continuo: pulso_hold)."""
        if hasattr(self, '_hilo_refresco') and self._hilo_refresco is not None:
            return
        self._refresco_activo = True

        def _bucle_refresco():
            servos_mantener = ('shoulder', 'elbow')
            while self._refresco_activo:
                for nombre in servos_mantener:
                    if nombre in self.servos:
                        servo = self.servos[nombre]
                        if self._es_servo_posicional(servo):
                            pulso = self._pulso_desde_posicion(
                                servo, servo['posicion_estimada'])
                        else:
                            pulso = servo['pulso_hold']
                        try:
                            self.pca.channels[servo['canal']].duty_cycle = self._us_a_duty(pulso)
                        except Exception:
                            pass
                time.sleep(intervalo)

        self._hilo_refresco = threading.Thread(target=_bucle_refresco, daemon=True)
        self._hilo_refresco.start()
        log.info(f"[Servo] Hilo anti-drift (intervalo={intervalo}s)")

    def detener_refresco_anti_drift(self):
        """Detiene el hilo de refresco anti-drift."""
        self._refresco_activo = False
        self._hilo_refresco = None

    def guardar_config(self):
        """Guarda la configuracion actual de servos a servo_config_legacy.json."""
        if not hasattr(self, '_config_path'):
            return

        nombres_tiempo = {
            'shoulder': ('tiempo_max_arriba', 'tiempo_max_abajo'),
            'elbow': ('tiempo_max_extender', 'tiempo_max_contraer'),
            'wrist': ('tiempo_max_horario', 'tiempo_max_antihorario'),
            'gripper': ('tiempo_max_abrir', 'tiempo_max_cerrar'),
            'base': ('tiempo_max_horario', 'tiempo_max_antihorario'),
        }

        config = {}
        for nombre, servo in self.servos.items():
            tp_key, tn_key = nombres_tiempo.get(nombre, ('tiempo_max_positivo', 'tiempo_max_negativo'))
            config[nombre] = {
                'canal': servo['canal'],
                'tipo_servo': servo.get('tipo_servo', 'posicional_180'),
                'pulso_min': servo['pulso_min'],
                'pulso_max': servo['pulso_max'],
                'pulso_neutral': servo['pulso_neutral'],
                'pulso_hold': servo['pulso_hold'],
                'rango_pulso': servo['rango_pulso'],
                'velocidad_min': servo['velocidad_min'],
                'invertido': servo.get('invertido', False),
                tp_key: round(servo['tiempo_max_positivo'], 2),
                tn_key: round(servo['tiempo_max_negativo'], 2),
            }
            if servo['pulso_abrir']:
                config[nombre]['pulso_abrir'] = servo['pulso_abrir']
            if servo['pulso_cerrar']:
                config[nombre]['pulso_cerrar'] = servo['pulso_cerrar']

        try:
            with open(self._config_path, 'w') as f:
                json.dump(config, f, indent=2)
            log.info(f"Configuracion guardada en {self._config_path}")
        except Exception as e:
            log.error(f"Error guardando servo_config_legacy.json: {e}")

    def detener_servo(self, nombre):
        """Mantiene la posición actual (180°) o pulso_hold (continuo)."""
        if nombre in self.servos:
            servo = self.servos[nombre]
            if self._es_servo_posicional(servo):
                pulso = self._pulso_desde_posicion(servo, servo['posicion_estimada'])
            else:
                pulso = servo['pulso_hold']
            self.pca.channels[servo['canal']].duty_cycle = self._us_a_duty(pulso)
            servo['ultimo_pulso'] = pulso
            log.info(f"[Servo] {nombre}: DETENIDO -> {pulso}us")

    def set_hold_after_move(self, enabled: bool, offset_us: int = None):
        """Habilitar/deshabilitar la aplicación de pequeño pulso de hold al terminar un movimiento.

        enabled: True para activar, False para desactivar.
        offset_us: si se pasa, actualiza self.hold_pulse_offset (microsegundos).
        """
        self.hold_after_move = bool(enabled)
        if offset_us is not None:
            try:
                self.hold_pulse_offset = int(offset_us)
            except Exception:
                log.warning("hold_pulse_offset debe ser entero (microsegundos); ignorando valor inválido")
        log.info(f"[Servo] set_hold_after_move={self.hold_after_move} hold_pulse_offset={self.hold_pulse_offset}")

    def detener_todos(self):
        """Detener todos los servos aplicando pulso_hold (neutral)."""
        for nombre in self.servos:
            self.detener_servo(nombre)

    def apagar_todos(self):
        """EMERGENCIA: corta la senal PWM de todos los canales (duty_cycle=0).
        Los servos quedan sin senal, se pueden mover libremente."""
        for nombre, servo in self.servos.items():
            try:
                self.pca.channels[servo['canal']].duty_cycle = 0
                servo['ultimo_pulso'] = 0
            except Exception:
                pass
        log.warning("[Servo] EMERGENCIA: Todas las senales PWM cortadas")

class ControladorStepper:
    """Controlador para motores stepper con perfil trapezoidal de aceleracion."""

    def __init__(self, pin_paso, pin_direccion, pin_habilitar=None,
                 pasos_por_rev=200, micropasos=16):
        from gpiozero import OutputDevice  # noqa: PLC0415

        self.pin_paso = OutputDevice(pin_paso)
        self.pin_direccion = OutputDevice(pin_direccion)
        self.pin_habilitar = OutputDevice(pin_habilitar) if pin_habilitar else None
        self.pasos_por_rev = pasos_por_rev * micropasos
        self.posicion_actual = 0

        self.vel_inicio = 200
        self.vel_max = 1000
        self.pasos_aceleracion_max = 150

    def habilitar(self):
        if self.pin_habilitar:
            self.pin_habilitar.off()

    def deshabilitar(self):
        if self.pin_habilitar:
            self.pin_habilitar.on()

    def mover_pasos(self, pasos, direccion=1, velocidad=None):
        """Mover stepper con perfil trapezoidal de aceleracion/desaceleracion.
        Si velocidad se especifica, se usa como vel_max para este movimiento."""
        pasos = abs(pasos)
        if pasos == 0:
            return

        self.habilitar()

        vel_max = velocidad if velocidad else self.vel_max
        vel_max = max(vel_max, self.vel_inicio + 1)

        self.pin_direccion.value = 1 if direccion > 0 else 0

        pasos_rampa = min(pasos // 3, self.pasos_aceleracion_max)
        pasos_rampa = max(pasos_rampa, 1)

        pasos_accel = pasos_rampa
        pasos_decel = pasos_rampa
        pasos_constante = pasos - pasos_accel - pasos_decel

        if pasos_constante < 0:
            pasos_accel = pasos // 2
            pasos_decel = pasos - pasos_accel
            pasos_constante = 0

        def _pulso(retardo):
            self.pin_paso.on()
            time.sleep(retardo / 2)
            self.pin_paso.off()
            time.sleep(retardo / 2)

        for i in range(pasos_accel):
            fraccion = (i + 1) / pasos_accel
            vel_actual = self.vel_inicio + (vel_max - self.vel_inicio) * fraccion
            _pulso(1.0 / vel_actual)

        if pasos_constante > 0:
            retardo_cte = 1.0 / vel_max
            for _ in range(pasos_constante):
                _pulso(retardo_cte)

        for i in range(pasos_decel):
            fraccion = 1.0 - (i + 1) / pasos_decel
            vel_actual = self.vel_inicio + (vel_max - self.vel_inicio) * fraccion
            _pulso(1.0 / max(vel_actual, self.vel_inicio))

        self.posicion_actual += pasos * (1 if direccion > 0 else -1)
        log.info(f"[Stepper] {pasos} pasos dir={direccion} vel_max={vel_max} "
                 f"rampa={pasos_accel}+{pasos_constante}+{pasos_decel} pos={self.posicion_actual}")

    def ir_a_posicion(self, pasos_destino, velocidad=None):
        """Mover a una posicion absoluta en pasos."""
        diferencia = pasos_destino - self.posicion_actual
        if diferencia == 0:
            return
        direccion = 1 if diferencia > 0 else -1
        self.mover_pasos(abs(diferencia), direccion, velocidad)

    def mover_distancia(self, distancia_mm, paso_tuerca=8, direccion=1, velocidad=None):
        pasos = int((distancia_mm / paso_tuerca) * self.pasos_por_rev)
        self.mover_pasos(pasos, direccion, velocidad)

class ControladorRobotico:
    """Controlador principal del brazo robótico con movimientos temporizados y límites físicos"""

    def __init__(self, habilitar_stepper=True):
        """Inicializar controlador del robot
        
        Args:
            habilitar_stepper: Si es False, no inicializa el motor paso a paso (útil si no está conectado o da error)
        """
        self.controlador_servo = ControladorServo()
        # Servos: hombro (0), codo (1), muñeca (2), pinza (3), base (4) — MG996R ~180° en base por defecto
        self.controlador_servo.agregar_servo('shoulder', 0)
        self.controlador_servo.agregar_servo('elbow', 1)
        self.controlador_servo.agregar_servo('wrist', 2)
        self.controlador_servo.agregar_servo('gripper', 3)
        self.controlador_servo.agregar_servo('base', 4)

        # Opcional: motor paso a paso (NEMA + TMC2208) si STEPPER_HABILITADO — GPIO17/18/19
        self.controlador_stepper = None
        if habilitar_stepper:
            try:
                self.controlador_stepper = ControladorStepper(
                    pin_paso=17, pin_direccion=18, pin_habilitar=19
                )
                self.controlador_stepper.habilitar()
                log.info("Motor paso a paso inicializado (GPIO17=STEP, GPIO18=DIR, GPIO19=EN)")
            except Exception as e:
                log.warning(f"⚠️  No se pudo inicializar motor paso a paso: {e}")
                log.warning("   El brazo funcionará solo con servos (sin movimiento horizontal)")
        else:
            log.info("ℹ️  Motor paso a paso deshabilitado (base por servo MG996R en canal 4)")

        # LÍMITES FÍSICOS DEL BRAZO (en segundos de movimiento)
        # Estos límites previenen que el brazo se salga de su rango físico
        self.limites_fisicos = {
            'base': {'izquierda': 3.0, 'derecha': 3.0},  # Máximo 3 segundos en cada dirección
            'shoulder': {'arriba': 2.5, 'abajo': 2.5},   # Máximo 2.5 segundos en cada dirección
            'elbow': {'extender': 3.5, 'contraer': 3.5}, # Máximo 3.5 segundos en cada dirección
            'gripper': {'abrir': 1.5, 'cerrar': 1.5}     # Máximo 1.5 segundos en cada dirección
        }

        # Estado actual de tiempo acumulado por articulación
        self.tiempo_acumulado = {
            'base': 0.0,
            'shoulder': 0.0,
            'elbow': 0.0,
            'gripper': 0.0
        }

    def mover_base_tiempo(self, direccion, tiempo_segundos, velocidad=0.5):
        """Base horizontal: prioriza stepper si está habilitado; si no, servo 'base' (canal 4)."""
        lim = self.limites_fisicos['base']['derecha' if direccion == 1 else 'izquierda']
        tiempo_limitado = min(tiempo_segundos, lim)
        if tiempo_limitado <= 0:
            return 0.0
        if self.controlador_stepper is None:
            if 'base' in self.controlador_servo.servos:
                self.controlador_servo.mover_por_tiempo(
                    'base', direccion, tiempo_limitado, velocidad)
            else:
                log.warning("Base: sin stepper ni servo 'base' — movimiento ignorado")
            self.tiempo_acumulado['base'] += tiempo_limitado * direccion
            return tiempo_limitado
        pasos = max(1, int(tiempo_limitado * (180 + 320 * max(0.15, min(1.0, velocidad)))))
        self.controlador_stepper.mover_pasos(pasos, direccion=direccion, velocidad=800)
        self.tiempo_acumulado['base'] += tiempo_limitado * direccion
        return tiempo_limitado

    def mover_hombro_tiempo(self, direccion, tiempo_segundos, velocidad=0.5):
        """Mover hombro por tiempo con límites físicos (velocidad reducida por defecto)"""
        tiempo_limitado = min(tiempo_segundos, self.limites_fisicos['shoulder']['arriba' if direccion == 1 else 'abajo'])
        if tiempo_limitado > 0:
            self.controlador_servo.mover_por_tiempo('shoulder', direccion, tiempo_limitado, velocidad)
            self.tiempo_acumulado['shoulder'] += tiempo_limitado * direccion
        return tiempo_limitado

    def mover_codo_tiempo(self, direccion, tiempo_segundos, velocidad=0.5):
        """Mover codo por tiempo con límites físicos (velocidad reducida por defecto)"""
        tiempo_limitado = min(tiempo_segundos, self.limites_fisicos['elbow']['extender' if direccion == 1 else 'contraer'])
        if tiempo_limitado > 0:
            self.controlador_servo.mover_por_tiempo('elbow', direccion, tiempo_limitado, velocidad)
            self.tiempo_acumulado['elbow'] += tiempo_limitado * direccion
        return tiempo_limitado

    def mover_pinza_tiempo(self, direccion, tiempo_segundos, velocidad=0.5):
        """Mover pinza por tiempo con límites físicos (velocidad reducida por defecto)"""
        tiempo_limitado = min(tiempo_segundos, self.limites_fisicos['gripper']['abrir' if direccion == 1 else 'cerrar'])
        if tiempo_limitado > 0:
            self.controlador_servo.mover_por_tiempo('gripper', direccion, tiempo_limitado, velocidad)
            self.tiempo_acumulado['gripper'] += tiempo_limitado * direccion
        return tiempo_limitado

    # MÉTODOS LEGACY PARA COMPATIBILIDAD (ya no se usan grados)
    def mover_base(self, angulo, velocidad=5):
        """Mover base del robot (LEGACY - ahora usa tiempo)"""
        log.warning("mover_base con ángulos está obsoleto. Usa mover_base_tiempo")
        # Convertir ángulo aproximado a tiempo (180° ≈ 2 segundos)
        tiempo = abs(angulo - 180) / 90.0  # Aproximación simple
        direccion = 1 if angulo > 180 else -1
        self.mover_base_tiempo(direccion, tiempo, velocidad)

    def mover_hombro(self, angulo, velocidad=5):
        """Mover hombro del robot (LEGACY)"""
        log.warning("mover_hombro con ángulos está obsoleto. Usa mover_hombro_tiempo")
        tiempo = abs(angulo - 180) / 90.0
        direccion = 1 if angulo > 180 else -1
        self.mover_hombro_tiempo(direccion, tiempo, velocidad)

    def mover_codo(self, angulo, velocidad=5):
        """Mover codo del robot (LEGACY)"""
        log.warning("mover_codo con ángulos está obsoleto. Usa mover_codo_tiempo")
        tiempo = abs(angulo - 180) / 90.0
        direccion = 1 if angulo > 180 else -1
        self.mover_codo_tiempo(direccion, tiempo, velocidad)

    def mover_pinza(self, angulo, velocidad=5):
        """Mover pinza del robot (LEGACY)"""
        log.warning("mover_pinza con ángulos está obsoleto. Usa mover_pinza_tiempo")
        tiempo = abs(angulo - 180) / 90.0
        direccion = 1 if angulo > 180 else -1
        self.mover_pinza_tiempo(direccion, tiempo, velocidad)

    def mover_brazo(self, distancia_mm, direccion=1, velocidad=1000):
        """Mover brazo horizontalmente (izquierda/derecha) usando motor paso a paso
        
        Args:
            distancia_mm: Distancia en milímetros a mover
            direccion: 1 = derecha, -1 = izquierda
            velocidad: Velocidad del motor (pasos por segundo)
        """
        if self.controlador_stepper is None:
            log.warning("⚠️  Motor paso a paso no disponible - movimiento horizontal deshabilitado")
            return
        self.controlador_stepper.mover_distancia(distancia_mm, direccion=direccion, velocidad=velocidad)

    def accion_recoger(self):
        """Abrir pinza para recoger"""
        self.mover_pinza_tiempo(1, 1.0)  # Abrir por 1 segundo

    def accion_soltar(self):
        """Cerrar pinza para soltar"""
        self.mover_pinza_tiempo(-1, 1.0)  # Cerrar por 1 segundo

    def mover_horizontal(self, distancia=50, direccion=1):
        """Mover brazo horizontalmente (izquierda/derecha) con motor paso a paso
        
        Args:
            distancia: Distancia en mm (por defecto 50mm)
            direccion: 1 = derecha, -1 = izquierda
        """
        self.mover_brazo(distancia, direccion=direccion)

    def resetear_tiempos(self):
        """Resetear contadores de tiempo acumulado"""
        self.tiempo_acumulado = {k: 0.0 for k in self.tiempo_acumulado}

    def obtener_estado_tiempos(self):
        """Obtener estado actual de tiempos acumulados"""
        return self.tiempo_acumulado.copy()

    def _mover_base_segun_pasos_legacy(self, pasos):
        """Rota la base: NEMA/stepper si está activo; si no, servo MG996R (escalado heurístico desde 'pasos' legacy)."""
        if pasos == 0:
            return
        direccion = 1 if pasos > 0 else -1
        if self.controlador_stepper is not None:
            self.controlador_stepper.mover_pasos(abs(pasos), direccion=direccion, velocidad=800)
            time.sleep(0.3)
        elif 'base' in self.controlador_servo.servos:
            t = min(3.0, max(0.12, abs(pasos) / 200.0 * 1.0))
            self.controlador_servo.mover_por_tiempo('base', direccion, t, 0.45)
            time.sleep(0.3)
        else:
            log.warning("Base: sin stepper ni servo 'base' — movimiento ignorado")

    def secuencia_recoger(self, angulo_base_pasos=0, tiempo_bajar=1.5, tiempo_cerrar=0.8, velocidad=0.4):
        """Secuencia completa de pick: posicionar, bajar, agarrar, subir.
        Retorna True si la secuencia se completo sin excepciones."""
        try:
            if angulo_base_pasos != 0:
                self._mover_base_segun_pasos_legacy(angulo_base_pasos)

            self.controlador_servo.mover_por_tiempo('shoulder', -1, tiempo_bajar, velocidad)
            time.sleep(0.2)
            self.controlador_servo.mover_por_tiempo('elbow', 1, tiempo_bajar * 0.6, velocidad)
            time.sleep(0.2)

            self.controlador_servo.mover_por_tiempo('gripper', -1, max(tiempo_cerrar, 0.8), velocidad=0.7)
            time.sleep(0.5)

            self.controlador_servo.mover_por_tiempo('shoulder', 1, tiempo_bajar * 1.1, velocidad)
            time.sleep(0.2)
            self.controlador_servo.mover_por_tiempo('elbow', -1, tiempo_bajar * 0.5, velocidad)

            log.info("Secuencia RECOGER completada")
            return True
        except Exception as e:
            log.error(f"Error en secuencia recoger: {e}")
            self._posicion_segura()
            return False

    def secuencia_soltar(self, angulo_base_pasos=0, tiempo_bajar=1.2, velocidad=0.4):
        """Secuencia completa de place: posicionar, bajar, soltar, subir."""
        try:
            if angulo_base_pasos != 0:
                self._mover_base_segun_pasos_legacy(angulo_base_pasos)

            self.controlador_servo.mover_por_tiempo('shoulder', -1, tiempo_bajar, velocidad)
            time.sleep(0.2)

            self.controlador_servo.mover_por_tiempo('gripper', 1, 1.0, velocidad=0.7)
            time.sleep(0.4)

            self.controlador_servo.mover_por_tiempo('shoulder', 1, tiempo_bajar * 1.1, velocidad)
            time.sleep(0.2)

            log.info("Secuencia SOLTAR completada")
            return True
        except Exception as e:
            log.error(f"Error en secuencia soltar: {e}")
            self._posicion_segura()
            return False

    def verificar_agarre(self):
        """Intenta detectar si la pinza realmente agarro algo.
        Hace un micro-cierre adicional: si la pinza se mueve muy rapido,
        probablemente no hay objeto (cerro en vacio)."""
        try:
            t_inicio = time.time()
            self.controlador_servo.mover_por_tiempo('gripper', -1, 0.15, velocidad=0.3)
            t_total = time.time() - t_inicio
            if t_total < 0.1:
                log.warning("Agarre posiblemente fallido: pinza cerro sin resistencia")
                return False
            log.info("Agarre verificado (resistencia detectada)")
            return True
        except Exception:
            return False

    def _posicion_segura(self):
        """Mover a posicion segura en caso de error."""
        log.warning("Moviendo a posicion segura...")
        try:
            self.controlador_servo.mover_por_tiempo('gripper', 1, 1.0, velocidad=0.7)
            time.sleep(0.3)
            self.controlador_servo.mover_por_tiempo('shoulder', 1, 1.5, velocidad=0.3)
            time.sleep(0.2)
            self.controlador_servo.mover_por_tiempo('elbow', -1, 1.0, velocidad=0.3)
            self.controlador_servo.detener_todos()
            log.info("Posicion segura alcanzada")
        except Exception as e:
            log.error(f"Error critico yendo a posicion segura: {e}")
            self.controlador_servo.detener_todos()

    def posicion_home(self, velocidad=0.5):
        """Mover todas las articulaciones a posicion de reposo."""
        try:
            self.controlador_servo.mover_por_tiempo('gripper', 1, 1.0, velocidad=0.7)
            time.sleep(0.2)
            self.controlador_servo.mover_por_tiempo('shoulder', 1, 1.5, velocidad)
            time.sleep(0.2)
            self.controlador_servo.mover_por_tiempo('elbow', -1, 1.0, velocidad)
            time.sleep(0.2)
            self.controlador_servo.mover_por_tiempo('wrist', 1, 0.5, velocidad)
            self.controlador_servo.detener_todos()
            self.resetear_tiempos()
            # No resetear posicion_estimada aquí: en servos 180° debe seguir al hardware
            log.info("Posicion HOME alcanzada")
        except Exception as e:
            log.error(f"Error en home: {e}")

    def posicion_escaneo(self, velocidad=0.3):
        """Posicionar brazo para que la camara tenga buena vista del area de trabajo."""
        try:
            self.controlador_servo.mover_por_tiempo('shoulder', 1, 0.8, velocidad)
            time.sleep(0.2)
            self.controlador_servo.mover_por_tiempo('elbow', 1, 0.5, velocidad)
            time.sleep(0.2)
            self.controlador_servo.mover_por_tiempo('wrist', -1, 0.3, velocidad)
            self.controlador_servo.detener_todos()
            log.info("Posicion de ESCANEO alcanzada")
        except Exception as e:
            log.error(f"Error en posicion escaneo: {e}")

    # --- Movimientos basados en objetivos angulares (helpers no intrusivos) ------
    def _deg_to_norm(self, deg: float) -> float:
        """Convierte grados (0..180) a posicion normalizada [0.0, 1.0]."""
        try:
            d = float(deg)
        except Exception:
            return 0.5
        return max(0.0, min(1.0, d / 180.0))

    def _move_servo_to_angle(self, nombre: str, target_deg: float, step_deg: float = 5.0, delay_between_steps: float = 0.02, velocidad: float = 0.5):
        """Mueve un servo posicional de forma progresiva hacia un objetivo en grados.

        Usa `mover_por_tiempo` en pasos pequeños calculando el tiempo necesario
        a partir de `tiempo_max_positivo`/`tiempo_max_negativo`.
        """
        if nombre not in self.controlador_servo.servos:
            log.warning(f"_move_servo_to_angle: servo '{nombre}' no configurado")
            return

        servo = self.controlador_servo.servos[nombre]
        if not self.controlador_servo._es_servo_posicional(servo):
            log.warning(f"_move_servo_to_angle: servo '{nombre}' no es posicional")
            return

        curr = float(servo.get('posicion_estimada', 0.5))
        target_norm = self._deg_to_norm(target_deg)
        delta = target_norm - curr
        if abs(delta) < 1e-4:
            return

        step_norm = abs(step_deg) / 180.0
        direction = 1 if delta > 0 else -1

        # Determinar tiempos maximos para la direccion
        t_max_pos = float(servo.get('tiempo_max_positivo', 1.0))
        t_max_neg = float(servo.get('tiempo_max_negativo', 1.0))

        remaining = abs(delta)
        while remaining > 1e-6:
            this_step = min(step_norm, remaining)
            t_max_dir = t_max_pos if direction > 0 else t_max_neg
            tiempo = max(0.0, this_step * t_max_dir)
            # En caso de tiempo casi 0, usar un mínimo pequeño para que haya efecto
            tiempo = max(tiempo, 0.005)
            try:
                self.controlador_servo.mover_por_tiempo(nombre, direction, tiempo, velocidad)
            except Exception as e:
                log.warning(f"Error moviendo servo '{nombre}' por tiempo: {e}")
                return
            time.sleep(delay_between_steps)
            remaining -= this_step

    def secuencia_personalizada_autonoma(self):
        """Secuencia autónoma personalizada (movimientos suaves y sincronizados).

        Ejecuta la secuencia descrita por el usuario: posiciones iniciales,
        barrido de base (0→180), sincronización con codo/muñeca, acciones de
        pinza y retorno, y fase de extensión final con ajuste dinámico de muñeca.
        """
        log.info("Iniciando secuencia personalizada autónoma")
        try:
            # 1) Posición inicial estable
            init = {
                'base': 10.0,
                'shoulder': 139.0,
                'elbow': 100.0,
                'wrist': 170.0,
            }
            for nombre, deg in init.items():
                self._move_servo_to_angle(nombre, deg, step_deg=5.0, delay_between_steps=0.03, velocidad=0.4)

            # Gripper a neutral/abierto
            try:
                self.controlador_servo.mover_por_tiempo('gripper', 1, 1.0, velocidad=0.6)
            except Exception:
                pass

            # 2) Barrido progresivo de la base: 0 -> 180 en pasos de 5°
            # Mientras tanto, el codo desciende a 80° sincronizado
            start_elbow = 100.0
            target_elbow = 80.0
            steps = list(range(0, 181, 5))
            # Asegurar base en 0 primero
            self._move_servo_to_angle('base', 0.0, step_deg=5.0, delay_between_steps=0.02, velocidad=0.45)

            # Sincronizar barrido de base y descenso del codo manteniendo HW_LOCK
            try:
                hw = _hw_bus
                if hw is not None and hw.HW_LOCK.acquire(timeout=1.0):
                    try:
                        # Calcular pasos normalizados
                        total_steps = max(1, len(steps))
                        for idx, deg in enumerate(steps):
                            prop = idx / max(1, total_steps - 1)
                            # Base: convertir a posicion normalizada
                            base_norm = max(0.0, min(1.0, deg / 180.0))
                            # Elbow: interpolar
                            elbow_deg = start_elbow + (target_elbow - start_elbow) * prop
                            elbow_norm = max(0.0, min(1.0, elbow_deg / 180.0))

                            # Aplicar pulsos directamente y actualizar posicion_estimada
                            try:
                                if 'base' in self.controlador_servo.servos:
                                    p_base = self.controlador_servo._pulso_desde_posicion(self.controlador_servo.servos['base'], base_norm)
                                    self.controlador_servo.aplicar_pulso('base', p_base)
                                    self.controlador_servo.servos['base']['posicion_estimada'] = base_norm
                                if 'elbow' in self.controlador_servo.servos:
                                    p_el = self.controlador_servo._pulso_desde_posicion(self.controlador_servo.servos['elbow'], elbow_norm)
                                    self.controlador_servo.aplicar_pulso('elbow', p_el)
                                    self.controlador_servo.servos['elbow']['posicion_estimada'] = elbow_norm
                            except Exception:
                                pass

                            # Pequeña espera para mantener movimiento suave
                            time.sleep(0.04)
                    finally:
                        try:
                            hw.HW_LOCK.release()
                        except Exception:
                            pass
                else:
                    # Fallback: si no podemos adquirir lock, hacemos movimientos individuales suaves
                    for deg in steps:
                        self._move_servo_to_angle('base', float(deg), step_deg=5.0, delay_between_steps=0.02, velocidad=0.45)
                        prop = deg / 180.0
                        elbow_deg = start_elbow + (target_elbow - start_elbow) * prop
                        self._move_servo_to_angle('elbow', elbow_deg, step_deg=3.0, delay_between_steps=0.01, velocidad=0.5)
            except Exception as e:
                log.warning(f"Sincronización barrido base/elbow falló: {e}")

            # 3) Muñeca descendiendo gradualmente hasta ~20°
            self._move_servo_to_angle('wrist', 20.0, step_deg=4.0, delay_between_steps=0.02, velocidad=0.45)

            # 4) Codo vuelve a elevarse lentamente hasta 92°
            self._move_servo_to_angle('elbow', 92.0, step_deg=2.0, delay_between_steps=0.03, velocidad=0.35)

            # 5) Pinza: secuencia rápida +360 then -360 (simulada por tiempos) + pausa de 3s
            try:
                self.controlador_servo.mover_por_tiempo('gripper', 1, 0.9, velocidad=1.0)
                time.sleep(0.15)
                self.controlador_servo.mover_por_tiempo('gripper', -1, 0.9, velocidad=1.0)
                time.sleep(3.0)  # Pausa para asegurar agarre
            except Exception:
                pass

            # 6) Después agarre: codo baja rápidamente a 80°, luego ajusta a 72°, hombro→114°
            self._move_servo_to_angle('elbow', 80.0, step_deg=6.0, delay_between_steps=0.02, velocidad=0.6)
            self._move_servo_to_angle('shoulder', 114.0, step_deg=5.0, delay_between_steps=0.03, velocidad=0.45)
            self._move_servo_to_angle('elbow', 72.0, step_deg=3.0, delay_between_steps=0.02, velocidad=0.4)

            # 7) Base retorna 180 -> 0 progresivo
            for deg in reversed(steps):
                self._move_servo_to_angle('base', float(deg), step_deg=5.0, delay_between_steps=0.02, velocidad=0.45)

            # 8) Fase de extension completa: hombro aumenta a 165°, codo disminuye a 34°
            shoulder_start = 114.0
            shoulder_target = 165.0
            elbow_start = 72.0
            elbow_target = 34.0
            total_steps_ext = int(abs(shoulder_target - shoulder_start) // 5) + 1
            for i in range(total_steps_ext + 1):
                prop = i / max(1, total_steps_ext)
                s_deg = shoulder_start + (shoulder_target - shoulder_start) * prop
                e_deg = elbow_start + (elbow_target - elbow_start) * prop
                # Muñeca compensa dinámicamente desde 20° hasta aproximadamente 155°
                w_deg = 20.0 + prop * 135.0
                self._move_servo_to_angle('shoulder', s_deg, step_deg=5.0, delay_between_steps=0.02, velocidad=0.45)
                self._move_servo_to_angle('elbow', e_deg, step_deg=5.0, delay_between_steps=0.02, velocidad=0.45)
                self._move_servo_to_angle('wrist', w_deg, step_deg=5.0, delay_between_steps=0.02, velocidad=0.4)

            # 9) Cuando extendido, pinza otra vez +360/-360 (simulada) + pausa de 3s
            try:
                self.controlador_servo.mover_por_tiempo('gripper', 1, 0.9, velocidad=1.0)
                time.sleep(0.15)
                self.controlador_servo.mover_por_tiempo('gripper', -1, 0.9, velocidad=1.0)
                time.sleep(3.0)  # Pausa para liberar objeto
            except Exception:
                pass

            log.info("Secuencia personalizada completada")
        except Exception as e:
            log.error(f"Error en secuencia personalizada: {e}")
            self._posicion_segura()

    def _calibrar_servo_180(self, nombre, servo, _notificar, idx, total, vel_cal):
        """Calibra tiempos de recorrido en servos ~180° (pasos hasta tope lógico 0/1)."""
        dt = 0.14
        max_iter = 60
        servo['posicion_estimada'] = 0.5

        log.info(f"  {nombre} (180°): barrido hacia min...")
        _notificar(nombre, 'tope_negativo', 0)
        t0 = time.time()
        for _ in range(max_iter):
            prev = servo['posicion_estimada']
            self.controlador_servo.mover_por_tiempo(nombre, -1, dt, vel_cal)
            if servo['posicion_estimada'] <= 0.02:
                break
            if abs(servo['posicion_estimada'] - prev) < 1e-6:
                break
        t_neg = time.time() - t0
        time.sleep(0.35)

        log.info(f"  {nombre} (180°): barrido hacia max...")
        _notificar(nombre, 'tope_positivo', 33)
        t0 = time.time()
        for _ in range(max_iter):
            prev = servo['posicion_estimada']
            self.controlador_servo.mover_por_tiempo(nombre, 1, dt, vel_cal)
            if servo['posicion_estimada'] >= 0.98:
                break
            if abs(servo['posicion_estimada'] - prev) < 1e-6:
                break
        t_pos = time.time() - t0
        time.sleep(0.35)

        servo['tiempo_max_positivo'] = round(max(0.55, t_pos * 1.12), 2)
        servo['tiempo_max_negativo'] = round(max(0.55, t_neg * 2.05), 2)

        log.info(f"  {nombre}: volviendo a centro (~0.5)...")
        _notificar(nombre, 'centro', 66)
        vuelta = max(0.35, min(servo['tiempo_max_negativo'] * 0.52, 4.0))
        self.controlador_servo.mover_por_tiempo(nombre, -1, vuelta, vel_cal)
        servo['posicion_estimada'] = 0.5
        pulso = self.controlador_servo._pulso_desde_posicion(servo, 0.5)
        self.controlador_servo.aplicar_pulso(nombre, pulso)

        log.info(f"  {nombre}: t_pos={servo['tiempo_max_positivo']:.2f}s "
                 f"t_neg={servo['tiempo_max_negativo']:.2f}s")
        _notificar(nombre, 'completado', int(((idx + 1) / total) * 100))
        time.sleep(0.25)

    def calibrar_inicio(self, callback=None):
        """Rutina de auto-calibracion: mueve cada servo a sus topes fisicos a velocidad
        baja, mide el tiempo total de recorrido y actualiza servo_config_legacy.json.

        Args:
            callback: funcion opcional callback(servo, fase, progreso) para reportar
                      progreso a la interfaz web.
        """
        def _notificar(servo, fase, progreso=0):
            if callback:
                try:
                    callback(servo, fase, progreso)
                except Exception:
                    pass

        servos_a_calibrar = ['shoulder', 'elbow', 'wrist']
        velocidad_calibracion = 0.3
        total = len(servos_a_calibrar)

        log.info("=== INICIO CALIBRACION AUTOMATICA ===")
        _notificar('sistema', 'inicio', 0)

        for idx, nombre in enumerate(servos_a_calibrar):
            servo = self.controlador_servo.servos.get(nombre)
            if not servo:
                continue

            log.info(f"Calibrando {nombre} ({idx+1}/{total})...")
            _notificar(nombre, 'inicio', int((idx / total) * 100))

            if ControladorServo._es_servo_posicional(servo):
                self._calibrar_servo_180(nombre, servo, _notificar, idx, total,
                                         velocidad_calibracion)
                continue

            servo['posicion_estimada'] = 0.5
            servo['tiempo_max_positivo'] = 30.0
            servo['tiempo_max_negativo'] = 30.0

            log.info(f"  {nombre}: moviendo a tope negativo...")
            _notificar(nombre, 'tope_negativo', 0)
            t_start = time.time()
            self.controlador_servo.mover_por_tiempo(nombre, -1, 8.0, velocidad_calibracion)
            t_negativo = time.time() - t_start
            time.sleep(0.5)

            log.info(f"  {nombre}: moviendo a tope positivo...")
            _notificar(nombre, 'tope_positivo', 33)
            t_start = time.time()
            self.controlador_servo.mover_por_tiempo(nombre, 1, 12.0, velocidad_calibracion)
            t_positivo = time.time() - t_start
            time.sleep(0.5)

            log.info(f"  {nombre}: volviendo a centro...")
            _notificar(nombre, 'centro', 66)
            t_medio = t_positivo / 2.0
            self.controlador_servo.mover_por_tiempo(nombre, -1, t_medio, velocidad_calibracion)

            margen = 1.15
            servo['tiempo_max_positivo'] = round(t_positivo * margen, 2)
            servo['tiempo_max_negativo'] = round(t_negativo * margen, 2)
            servo['posicion_estimada'] = 0.5

            log.info(f"  {nombre}: t_pos={servo['tiempo_max_positivo']:.2f}s "
                     f"t_neg={servo['tiempo_max_negativo']:.2f}s")
            _notificar(nombre, 'completado', int(((idx + 1) / total) * 100))
            time.sleep(0.3)

        self.controlador_servo.guardar_config()
        self.controlador_servo.resetear_posiciones()

        log.info("=== CALIBRACION COMPLETADA ===")
        _notificar('sistema', 'completado', 100)

    def cerrar(self):
        """Cerrar controladores y liberar recursos"""
        try:
            self.controlador_servo.detener_todos()
            if self.controlador_stepper:
                self.controlador_stepper.deshabilitar()
            self.controlador_servo.pca.deinit()
        except Exception as e:
            log.error(f"Error cerrando controladores: {e}")


# Export explícito y comprobación de integridad (evita copias truncadas en la Pi).
__all__ = ("ControladorServo", "ControladorStepper", "ControladorRobotico")
if "ControladorRobotico" not in globals():
    raise RuntimeError(
        "robot_controller.py incompleto o corrupto: falta la clase ControladorRobotico "
        "en el espacio de nombres del módulo. Vuelve a copiar el archivo desde el repo "
        "y comprueba que no exista la carpeta arm_system/control/robot_controller/ "
        "(debe ser solo el archivo .py)."
    )
