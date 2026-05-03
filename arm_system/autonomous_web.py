#!/usr/bin/env python3
"""
Interfaz web para el modo autonomo del brazo robotico.
Muestra video en vivo con detecciones superpuestas,
estado del robot, estadisticas, posiciones estimadas de servos,
y controles de calibracion/pausa/stop/resume.
"""

import html
import os
import sys
import time
import threading
import logging as log
import json
from datetime import datetime
from pathlib import Path

from flask import Flask, Response, request, jsonify, render_template_string

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from autonomous_brain import CerebroAutonomo
from config_sistema import (
    VOZ_HABILITADA,
    VOZ_IDIOMA_RECONOCIMIENTO,
    VOZ_ANUNCIAR_EVENTOS,
    VOZ_MIC_DEVICE_INDEX,
)
from safety.safe_controller import SafeController

log.basicConfig(level=log.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

# Grados por pulsación de botón en control manual (SafeController)
PASO_MANUAL_DEG: float = 10.0

# ------------------------------------------------------------------ #
# CALIBRACIÓN SEGURA (núcleo)
# ------------------------------------------------------------------ #
CAL_STEP_DEG: float = 1.0
CAL_STEP_DELAY_S: float = 0.05
CAL_COOLDOWN_MS: int = 120
CAL_MAX_JUMP = {
    'shoulder': 8.0,
    'elbow': 8.0,
    'base': 12.0,
    'wrist': 12.0,
}
CAL_JOINTS = ('base', 'shoulder', 'elbow', 'wrist')

_calibration_lock = threading.Lock()
_calibration_move_lock = threading.Lock()
_calibration_state = {
    'enabled': False,
    'active_joint': 'shoulder',
    'runtime': 'IDLE',  # IDLE | MOVING | STOPPED | EMERGENCY
    'last_cmd_ts_ms': 0,
    'stop_requested': False,
    'limits': {j: {'min': None, 'max': None} for j in CAL_JOINTS},
}

_calib_log = log.getLogger('calibration')
if not _calib_log.handlers:
    _calib_log.setLevel(log.INFO)
    _calib_log.propagate = False
    _logs_dir = Path(__file__).resolve().parent.parent / 'logs'
    _logs_dir.mkdir(parents=True, exist_ok=True)
    _fh = log.FileHandler(_logs_dir / 'calibration.log', encoding='utf-8')
    _fh.setFormatter(log.Formatter('%(asctime)s [%(levelname)s] %(message)s'))
    _calib_log.addHandler(_fh)

app = Flask(__name__)
cerebro = None
hilo_autonomo = None
hilo_calibracion = None
estado_calibracion = {'activo': False, 'servo': '', 'fase': '', 'progreso': 0}
_asistente_voz = None
_safe_ctrl: SafeController = None


def obtener_cerebro():
    global cerebro
    if cerebro is None:
        cerebro = CerebroAutonomo(habilitar_hardware=True)
    return cerebro


def obtener_safe_ctrl() -> SafeController:
    """
    Retorna (creando si es necesario) el SafeController para control manual.

    SafeController usa ArmController (ángulos, servo_config.json).
    ControladorRobotico (tiempo, modo autónomo) es un subsistema separado.
    La exclusión mutua sobre el PCA9685 se garantiza a través de hw_bus.HW_LOCK,
    compartido entre SafeController y ControladorServo.mover_por_tiempo().
    """
    global _safe_ctrl
    if _safe_ctrl is None:
        _safe_ctrl = SafeController()
    return _safe_ctrl


def _anunciar_voz(frase: str) -> None:
    if not (VOZ_HABILITADA and VOZ_ANUNCIAR_EVENTOS and _asistente_voz):
        return
    try:
        _asistente_voz.voz.hablar(frase)
    except Exception:
        pass


def _is_calibration_enabled() -> bool:
    with _calibration_lock:
        return bool(_calibration_state['enabled'])


def _calibration_blocked_response(endpoint_name: str):
    return jsonify({
        'ok': False,
        'msg': f'Calibración segura activa. Endpoint bloqueado: {endpoint_name}'
    }), 423


def _calibration_event(action: str, joint: str = '', from_angle=None, to_angle=None, extra: str = '') -> None:
    _calib_log.info(
        "action=%s joint=%s from=%s to=%s extra=%s",
        action,
        joint or '-',
        '-' if from_angle is None else f"{float(from_angle):.2f}",
        '-' if to_angle is None else f"{float(to_angle):.2f}",
        extra or '-',
    )


def _force_gripper_neutral() -> None:
    """
    Durante calibración, la garra continua debe quedar neutral o detenida.
    """
    try:
        c = obtener_cerebro()
        c.robot.controlador_servo.detener_servo('gripper')
    except Exception:
        pass


def _calibration_abort_requested() -> bool:
    """
    True cuando STOP/EMERGENCY pidió abortar inmediatamente el movimiento.
    """
    with _calibration_lock:
        return bool(_calibration_state['stop_requested']) or _calibration_state['runtime'] == 'EMERGENCY'


def _calibration_step(direction: int):
    if direction not in (-1, 1):
        return {'ok': False, 'msg': 'direction inválido (usar -1 o 1)'}

    now_ms = int(time.time() * 1000)
    with _calibration_lock:
        if not _calibration_state['enabled']:
            return {'ok': False, 'msg': 'Calibración no activa'}
        if _calibration_state['runtime'] == 'EMERGENCY':
            return {'ok': False, 'msg': 'Calibración en EMERGENCY. Resetea emergencia primero.'}
        elapsed = now_ms - int(_calibration_state['last_cmd_ts_ms'])
        if elapsed < CAL_COOLDOWN_MS:
            return {'ok': False, 'msg': f'Cooldown activo ({elapsed} ms)'}
        joint = str(_calibration_state['active_joint'])
        _calibration_state['last_cmd_ts_ms'] = now_ms
        _calibration_state['runtime'] = 'MOVING'
        _calibration_state['stop_requested'] = False

    if not _calibration_move_lock.acquire(blocking=False):
        with _calibration_lock:
            _calibration_state['runtime'] = 'STOPPED'
        return {'ok': False, 'msg': 'Movimiento en curso. Rechazado por lock de calibración.'}

    safe = obtener_safe_ctrl()
    try:
        # STOP/EMERGENCY recibido justo antes de tomar el lock de movimiento.
        if _calibration_abort_requested():
            with _calibration_lock:
                _calibration_state['runtime'] = 'STOPPED'
            return {'ok': False, 'msg': 'Movimiento abortado por STOP/EMERGENCY'}

        if safe.is_emergency:
            with _calibration_lock:
                _calibration_state['runtime'] = 'EMERGENCY'
            return {'ok': False, 'msg': 'SafeController en emergency stop.'}

        current = safe.get_angle(joint)
        target = current + direction * CAL_STEP_DEG
        max_jump = CAL_MAX_JUMP.get(joint, 8.0)
        if abs(target - current) > max_jump:
            _calibration_event('CAL_STEP_REJECTED_JUMP', joint, current, target, f'max_jump={max_jump}')
            with _calibration_lock:
                _calibration_state['runtime'] = 'STOPPED'
            return {'ok': False, 'msg': f'Salto rechazado por seguridad (>{max_jump}°)'}

        # STOP/EMERGENCY antes de escribir PWM vía move_safe
        if _calibration_abort_requested():
            with _calibration_lock:
                _calibration_state['runtime'] = 'STOPPED'
            _calibration_event('CAL_STEP_ABORT_PRE_PWM', joint, current, target)
            return {'ok': False, 'msg': 'Abortado antes de PWM por STOP/EMERGENCY'}

        ok = safe.move_safe(joint, target)

        # STOP/EMERGENCY mientras move_safe retornaba
        if _calibration_abort_requested():
            with _calibration_lock:
                _calibration_state['runtime'] = 'STOPPED'
            _calibration_event('CAL_STEP_ABORT_POST_MOVE', joint, current, safe.get_angle(joint))
            return {'ok': False, 'msg': 'Abortado por STOP/EMERGENCY'}

        time.sleep(CAL_STEP_DELAY_S)

        # STOP/EMERGENCY después del delay de asentamiento
        if _calibration_abort_requested():
            with _calibration_lock:
                _calibration_state['runtime'] = 'STOPPED'
            _calibration_event('CAL_STEP_ABORT_POST_DELAY', joint, current, safe.get_angle(joint))
            return {'ok': False, 'msg': 'Abortado tras delay por STOP/EMERGENCY'}

        applied = safe.get_angle(joint)
        _calibration_event('CAL_STEP', joint, current, applied, f'dir={direction}')

        with _calibration_lock:
            _calibration_state['runtime'] = 'IDLE'

        if not ok:
            return {'ok': False, 'msg': 'Movimiento rechazado por SafeController'}
        return {'ok': True, 'msg': f'{joint}: {current:.1f}° -> {applied:.1f}°', 'angle': applied}
    finally:
        _calibration_move_lock.release()


def iniciar_modo_autonomo(ciclos: int = 50) -> tuple:
    """Lógica compartida: API y comandos de voz."""
    global hilo_autonomo
    c = obtener_cerebro()
    if hilo_autonomo and hilo_autonomo.is_alive():
        return False, 'Ya esta ejecutandose'
    c._detener = False
    c._pausar = False
    hilo_autonomo = threading.Thread(
        target=c.ejecutar_ciclo_autonomo, args=(ciclos,), daemon=True)
    hilo_autonomo.start()
    _anunciar_voz('Modo autónomo iniciado.')
    return True, 'Modo autonomo iniciado'


def _registrar_voz_si_habilitada() -> None:
    global _asistente_voz
    if not VOZ_HABILITADA:
        return
    try:
        from voice_assistant import AsistenteVoz
    except ImportError as e:
        log.warning('Voz deshabilitada: %s', e)
        return

    def _iniciar():
        ok, _ = iniciar_modo_autonomo(50)
        if not ok:
            _asistente_voz.voz.hablar('El modo autónomo ya estaba en marcha.')

    def _pausar():
        obtener_cerebro().pausar()
        _anunciar_voz('Pausado.')

    def _reanudar():
        obtener_cerebro().reanudar()
        _anunciar_voz('Reanudado.')

    def _detener():
        c = obtener_cerebro()
        c.detener()
        c.robot.controlador_servo.detener_todos()
        _anunciar_voz('Detenido.')

    def _home():
        obtener_cerebro().robot.posicion_home()
        _anunciar_voz('Posición home.')

    def _escanear():
        c = obtener_cerebro()
        c._escanear_entorno()
        _anunciar_voz('Escaneo hecho.')

    def _emergencia():
        c = obtener_cerebro()
        c.detener()
        c.robot.controlador_servo.apagar_todos()
        if c.robot.controlador_stepper:
            c.robot.controlador_stepper.deshabilitar()
        c.robot.resetear_tiempos()
        _asistente_voz.voz.hablar('Parada de emergencia.')

    def _calibrar_servos():
        global hilo_calibracion
        if hilo_calibracion and hilo_calibracion.is_alive():
            _asistente_voz.voz.hablar('Calibración de servos ya en curso.')
            return
        c = obtener_cerebro()

        def _callback(servo, fase, progreso):
            estado_calibracion['activo'] = fase != 'completado' or servo != 'sistema'
            estado_calibracion['servo'] = servo
            estado_calibracion['fase'] = fase
            estado_calibracion['progreso'] = progreso

        def _ejecutar():
            estado_calibracion['activo'] = True
            try:
                c.robot.calibrar_inicio(callback=_callback)
            except Exception as e:
                log.error('Error en calibracion: %s', e)
            finally:
                estado_calibracion['activo'] = False

        hilo_calibracion = threading.Thread(target=_ejecutar, daemon=True)
        hilo_calibracion.start()
        _anunciar_voz('Iniciando calibración de servos.')

    def _calibrar_color():
        c = obtener_cerebro()
        if c.detector_color is None:
            _asistente_voz.voz.hablar('Detector de color no disponible.')
            return
        img = c._capturar_imagen()
        if img is None:
            _asistente_voz.voz.hablar('No hay imagen para calibrar color.')
            return
        c.detector_color.calibrar_iluminacion(img)
        _anunciar_voz('Color calibrado.')

    acciones = {
        'iniciar': _iniciar,
        'pausar': _pausar,
        'reanudar': _reanudar,
        'detener': _detener,
        'home': _home,
        'escanear': _escanear,
        'emergencia': _emergencia,
        'calibrar_servos': _calibrar_servos,
        'calibrar_color': _calibrar_color,
    }
    _asistente_voz = AsistenteVoz(
        acciones,
        idioma_google=VOZ_IDIOMA_RECONOCIMIENTO,
        mic_device_index=VOZ_MIC_DEVICE_INDEX,
    )
    if _asistente_voz.iniciar():
        log.info('Comandos de voz activos.')
    else:
        _asistente_voz = None


# ------------------------------------------------------------------ #
# VIDEO STREAM
# ------------------------------------------------------------------ #

def generar_frames():
    """Genera frames MJPEG continuos desde la camara."""
    import cv2
    c = obtener_cerebro()
    while True:
        try:
            img = c._capturar_imagen()
            if img is not None:
                c.frame_actual = img.copy()

                if c.detector_color:
                    objs_draw = [{'bbox': o.bbox, 'color': o.color,
                                  'clase': o.clase, 'confianza': o.confianza}
                                 for o in c.objetos] if c.objetos else []
                    recs_draw = [{'bbox': r.bbox, 'color': r.color,
                                  'centro': r.centro}
                                 for r in c.recipientes] if c.recipientes else []
                    if objs_draw or recs_draw:
                        img = c.detector_color.dibujar_resultados(img, objs_draw, recs_draw)

                _, buffer = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            else:
                time.sleep(1)
        except Exception:
            time.sleep(1)

        time.sleep(0.15)


# ------------------------------------------------------------------ #
# RUTAS API
# ------------------------------------------------------------------ #

@app.route('/video_feed')
def video_feed():
    return Response(generar_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/api/estado')
def api_estado():
    c = obtener_cerebro()
    estado = c.obtener_estado()
    try:
        estado['posiciones'] = c.robot.controlador_servo.obtener_posiciones_estimadas()
    except Exception:
        estado['posiciones'] = {}
    estado['calibracion'] = estado_calibracion.copy()
    estado['voz_activa'] = bool(_asistente_voz)
    estado['voz_config_habilitada'] = VOZ_HABILITADA

    # Estado del SafeController (capa de control seguro)
    safe = obtener_safe_ctrl()
    estado['safe_emergency'] = safe.is_emergency
    estado['safe_simulation'] = safe.is_simulation
    estado['safe_angles'] = safe.get_all_angles()
    with _calibration_lock:
        estado['calibration_mode'] = bool(_calibration_state['enabled'])
        estado['calibration_runtime'] = str(_calibration_state['runtime'])
        estado['calibration_active_joint'] = str(_calibration_state['active_joint'])
        estado['calibration_limits'] = json.loads(json.dumps(_calibration_state['limits']))

    return jsonify(estado)


@app.route('/api/iniciar', methods=['POST'])
def api_iniciar():
    if _is_calibration_enabled():
        return _calibration_blocked_response('/api/iniciar')
    ciclos = request.json.get('ciclos', 50) if request.is_json else 50
    ok, msg = iniciar_modo_autonomo(ciclos)
    return jsonify({'ok': ok, 'msg': msg})


@app.route('/api/pausar', methods=['POST'])
def api_pausar():
    if _is_calibration_enabled():
        return _calibration_blocked_response('/api/pausar')
    obtener_cerebro().pausar()
    _anunciar_voz('Pausado.')
    return jsonify({'ok': True, 'msg': 'Pausado'})


@app.route('/api/reanudar', methods=['POST'])
def api_reanudar():
    if _is_calibration_enabled():
        return _calibration_blocked_response('/api/reanudar')
    obtener_cerebro().reanudar()
    _anunciar_voz('Reanudado.')
    return jsonify({'ok': True, 'msg': 'Reanudado'})


@app.route('/api/detener', methods=['POST'])
def api_detener():
    if _is_calibration_enabled():
        return _calibration_blocked_response('/api/detener')
    c = obtener_cerebro()
    c.detener()
    c.robot.controlador_servo.detener_todos()
    _anunciar_voz('Detenido.')
    return jsonify({'ok': True, 'msg': 'Detenido - servos en hold'})


@app.route('/api/home', methods=['POST'])
def api_home():
    if _is_calibration_enabled():
        return _calibration_blocked_response('/api/home')
    safe = obtener_safe_ctrl()
    if safe.is_emergency:
        return jsonify({'ok': False, 'msg': 'Emergency stop activo.'})
    # go_home() usa move_safe() que compite por HW_LOCK: si el modo autónomo
    # está en medio de un comando, el primer paso de go_home() será rechazado.
    ok = safe.go_home()
    if not ok:
        return jsonify({'ok': False, 'msg': 'go_home() rechazado (hardware ocupado o emergency).'})
    _anunciar_voz('Posición home.')
    return jsonify({'ok': True, 'msg': 'Posicion HOME'})


@app.route('/api/escanear', methods=['POST'])
def api_escanear():
    if _is_calibration_enabled():
        return _calibration_blocked_response('/api/escanear')
    c = obtener_cerebro()
    objs, recs = c._escanear_entorno()
    return jsonify({
        'ok': True,
        'objetos': [o.to_dict() for o in objs],
        'recipientes': [r.to_dict() for r in recs],
    })


@app.route('/api/mover', methods=['POST'])
def api_mover():
    if _is_calibration_enabled():
        return _calibration_blocked_response('/api/mover')
    """
    Control manual por ángulos relativos via SafeController.
    Cuerpo: {'joint': 'shoulder', 'dir': 1}
      dir = +1 (positivo) / -1 (negativo) / 0 (sin movimiento)
    Cada pulsación mueve PASO_MANUAL_DEG grados en la dirección indicada.

    La exclusión mutua con el modo autónomo la gestiona hw_bus.HW_LOCK:
    si ControladorServo tiene el lock (ejecutando un comando autónomo),
    SafeController.move_relative() retornará False y el endpoint lo informa al cliente.
    """
    from arm_system import hw_bus

    data = request.get_json() or {}
    joint = data.get('joint', 'shoulder')
    direccion = int(data.get('dir', 0))

    if direccion == 0:
        return jsonify({'ok': True, 'msg': 'Sin movimiento (dir=0)'})

    safe = obtener_safe_ctrl()

    if safe.is_emergency:
        return jsonify({
            'ok': False,
            'msg': 'Emergency stop activo. Usar /api/reset_emergency para reanudar.'
        })

    # Verificación rápida (no bloqueante) de disponibilidad del bus.
    # Es informativa: el rechazo definitivo ocurre dentro de move_relative()
    # vía HW_LOCK.acquire(timeout=...) en el loop de interpolación.
    if not hw_bus.HW_LOCK.acquire(blocking=False):
        return jsonify({
            'ok': False,
            'msg': 'Hardware ocupado (modo autónomo en ejecución). Reintenta en un momento.'
        })
    hw_bus.HW_LOCK.release()

    # Caso especial: garra continua 360 (legacy)
    # La garra se controla por dirección/tiempo (abrir/cerrar), no por ángulo absoluto.
    if joint == 'gripper':
        try:
            c = obtener_cerebro()
            c.robot.controlador_servo.mover_por_tiempo('gripper', direccion, 0.25, velocidad=0.45)
            return jsonify({
                'ok': True,
                'msg': f'gripper continuo movido dir={direccion:+d} (0.25s)'
            })
        except Exception as exc:
            return jsonify({
                'ok': False,
                'msg': f'Error moviendo gripper continuo: {exc}'
            })

    delta = direccion * PASO_MANUAL_DEG
    ok = safe.move_relative(joint, delta)

    if ok:
        return jsonify({
            'ok': True,
            'msg': f'{joint} movido {delta:+.0f}° → {safe.get_angle(joint):.1f}°'
        })
    else:
        return jsonify({'ok': False, 'msg': 'Movimiento rechazado por SafeController'})


@app.route('/api/set_angle', methods=['POST'])
def api_set_angle():
    if _is_calibration_enabled():
        return _calibration_blocked_response('/api/set_angle')
    """
    Control absoluto por ángulo (sliders) para articulaciones posicionales.
    Cuerpo: {'joint': 'shoulder', 'angle': 92.0}
    """
    data = request.get_json() or {}
    joint = str(data.get('joint', '')).strip().lower()

    if joint not in {'base', 'shoulder', 'elbow', 'wrist'}:
        return jsonify({'ok': False, 'msg': f'Joint no soportado para slider: {joint}'})

    try:
        angle = float(data.get('angle'))
    except Exception:
        return jsonify({'ok': False, 'msg': 'Ángulo inválido'})

    safe = obtener_safe_ctrl()
    if safe.is_emergency:
        return jsonify({'ok': False, 'msg': 'Emergency stop activo. Reinicia antes de mover.'})

    # El slider puede saltar varios grados en un solo evento.
    # Para respetar el límite anti-salto del SafeController, aplicamos una rampa.
    current = safe.get_angle(joint)
    delta = angle - current
    max_step = 10.0 if joint in {'shoulder', 'elbow'} else 15.0

    if abs(delta) <= max_step:
        ok = safe.move_safe(joint, angle)
        if not ok:
            return jsonify({'ok': False, 'msg': 'Movimiento rechazado por SafeController'})
    else:
        direction = 1.0 if delta > 0 else -1.0
        steps = int(abs(delta) / max_step)
        last_target = current
        for _ in range(steps):
            last_target += direction * max_step
            if not safe.move_safe(joint, last_target):
                return jsonify({'ok': False, 'msg': f'Movimiento rechazado en rampa ({joint})'})
            # Rate limit de SafeController: 80 ms
            time.sleep(0.09)
        if abs(last_target - angle) > 1e-6:
            if not safe.move_safe(joint, angle):
                return jsonify({'ok': False, 'msg': f'Movimiento final rechazado ({joint})'})

    applied = safe.get_angle(joint)
    return jsonify({'ok': True, 'msg': f'{joint} => {applied:.1f}°', 'angle_applied': applied})


@app.route('/api/gripper_continuo', methods=['POST'])
def api_gripper_continuo():
    if _is_calibration_enabled():
        return _calibration_blocked_response('/api/gripper_continuo')
    """
    Control de garra continua 360 por velocidad.
    Cuerpo: {'speed': -100..100}
      speed > 0 : abrir
      speed < 0 : cerrar
      speed = 0 : detener
    """
    data = request.get_json() or {}
    try:
        speed = int(data.get('speed', 0))
    except Exception:
        return jsonify({'ok': False, 'msg': 'speed inválido'})

    speed = max(-100, min(100, speed))
    c = obtener_cerebro()

    try:
        if speed == 0:
            c.robot.controlador_servo.detener_servo('gripper')
            return jsonify({'ok': True, 'msg': 'gripper continuo detenido'})

        direccion = 1 if speed > 0 else -1
        velocidad = max(0.2, min(1.0, abs(speed) / 100.0))
        c.robot.controlador_servo.mover_por_tiempo('gripper', direccion, 0.12, velocidad=velocidad)
        return jsonify({'ok': True, 'msg': f'gripper continuo speed={speed}'})
    except Exception as exc:
        return jsonify({'ok': False, 'msg': f'Error en gripper continuo: {exc}'})


@app.route('/api/emergencia', methods=['POST'])
def api_emergencia():
    """
    Parada de emergencia total: detiene AMBOS subsistemas.
    1. SafeController.emergency_stop() — corta PWM del ArmController (control manual)
    2. ControladorRobotico.apagar_todos() — corta PWM del subsistema autónomo
    """
    # Detener SafeController (control manual / ArmController)
    obtener_safe_ctrl().emergency_stop()

    # Detener ControladorRobotico (modo autónomo / secuencias)
    c = obtener_cerebro()
    c.detener()
    c.robot.controlador_servo.apagar_todos()
    if c.robot.controlador_stepper:
        c.robot.controlador_stepper.deshabilitar()
    c.robot.resetear_tiempos()

    if VOZ_HABILITADA and _asistente_voz:
        _asistente_voz.voz.hablar('Parada de emergencia.')
    return jsonify({'ok': True, 'msg': 'PARADA DE EMERGENCIA - Todos los servos apagados'})


@app.route('/api/reset_emergency', methods=['POST'])
def api_reset_emergency():
    """
    Reinicia el emergency stop del SafeController.
    Solo llamar cuando el brazo esté en posición segura y haya sido
    inspeccionado físicamente por el operador.
    """
    safe = obtener_safe_ctrl()
    if not safe.is_emergency:
        return jsonify({'ok': True, 'msg': 'No había emergency stop activo'})
    safe.reset_emergency()
    with _calibration_lock:
        if _calibration_state['enabled'] and _calibration_state['runtime'] == 'EMERGENCY':
            # Reset manual explícito: salimos de EMERGENCY, pero quedamos en STOPPED
            # para evitar reanudación accidental.
            _calibration_state['runtime'] = 'STOPPED'
            _calibration_state['stop_requested'] = False
    log.warning('[Web] Emergency stop reiniciado por operador desde /api/reset_emergency')
    return jsonify({'ok': True, 'msg': 'Emergency stop reiniciado. Verificar posición del brazo.'})


@app.route('/api/calibration/enable', methods=['POST'])
def api_calibration_enable():
    with _calibration_lock:
        _calibration_state['enabled'] = True
        _calibration_state['runtime'] = 'IDLE'
        _calibration_state['stop_requested'] = False
        _calibration_state['last_cmd_ts_ms'] = 0
    _force_gripper_neutral()
    _calibration_event('CAL_ENABLE', extra='calibration_mode=True')
    return jsonify({'ok': True, 'msg': 'Modo calibración seguro ACTIVADO'})


@app.route('/api/calibration/disable', methods=['POST'])
def api_calibration_disable():
    with _calibration_lock:
        _calibration_state['enabled'] = False
        _calibration_state['runtime'] = 'IDLE'
        _calibration_state['stop_requested'] = False
    _force_gripper_neutral()
    _calibration_event('CAL_DISABLE', extra='calibration_mode=False')
    return jsonify({'ok': True, 'msg': 'Modo calibración seguro DESACTIVADO'})


@app.route('/api/calibration/select_joint', methods=['POST'])
def api_calibration_select_joint():
    data = request.get_json() or {}
    joint = str(data.get('joint', '')).strip().lower()
    if joint not in CAL_JOINTS:
        return jsonify({'ok': False, 'msg': f'Joint inválido: {joint}'})
    with _calibration_lock:
        if not _calibration_state['enabled']:
            return jsonify({'ok': False, 'msg': 'Activa calibración primero'})
        _calibration_state['active_joint'] = joint
        _calibration_state['runtime'] = 'IDLE'
    _calibration_event('CAL_SELECT_JOINT', joint=joint)
    return jsonify({'ok': True, 'msg': f'Joint activo: {joint}'})


@app.route('/api/calibration/step', methods=['POST'])
def api_calibration_step():
    data = request.get_json() or {}
    try:
        direction = int(data.get('dir', 0))
    except Exception:
        return jsonify({'ok': False, 'msg': 'dir inválido'})
    out = _calibration_step(direction)
    return jsonify(out), (200 if out.get('ok') else 409)


@app.route('/api/calibration/stop', methods=['POST'])
def api_calibration_stop():
    with _calibration_lock:
        _calibration_state['stop_requested'] = True
        if _calibration_state['runtime'] != 'EMERGENCY':
            _calibration_state['runtime'] = 'STOPPED'
    _force_gripper_neutral()
    _calibration_event('CAL_STOP')
    return jsonify({'ok': True, 'msg': 'STOP de calibración aplicado'})


@app.route('/api/calibration/emergency', methods=['POST'])
def api_calibration_emergency():
    with _calibration_lock:
        _calibration_state['runtime'] = 'EMERGENCY'
        _calibration_state['stop_requested'] = True
    _force_gripper_neutral()
    _calibration_event('CAL_EMERGENCY')
    # Reusar emergencia global para máxima seguridad física.
    return api_emergencia()


@app.route('/api/calibration/save_min', methods=['POST'])
def api_calibration_save_min():
    safe = obtener_safe_ctrl()
    with _calibration_lock:
        if not _calibration_state['enabled']:
            return jsonify({'ok': False, 'msg': 'Activa calibración primero'})
        joint = _calibration_state['active_joint']
    angle = safe.get_angle(joint)
    with _calibration_lock:
        _calibration_state['limits'][joint]['min'] = round(angle, 2)
    _calibration_event('CAL_SAVE_MIN', joint=joint, to_angle=angle)
    return jsonify({'ok': True, 'msg': f'MIN guardado {joint}={angle:.2f}°'})


@app.route('/api/calibration/save_max', methods=['POST'])
def api_calibration_save_max():
    safe = obtener_safe_ctrl()
    with _calibration_lock:
        if not _calibration_state['enabled']:
            return jsonify({'ok': False, 'msg': 'Activa calibración primero'})
        joint = _calibration_state['active_joint']
    angle = safe.get_angle(joint)
    with _calibration_lock:
        _calibration_state['limits'][joint]['max'] = round(angle, 2)
    _calibration_event('CAL_SAVE_MAX', joint=joint, to_angle=angle)
    return jsonify({'ok': True, 'msg': f'MAX guardado {joint}={angle:.2f}°'})


@app.route('/api/calibration/commit', methods=['POST'])
def api_calibration_commit():
    with _calibration_lock:
        limits_snapshot = json.loads(json.dumps(_calibration_state['limits']))

    for joint in CAL_JOINTS:
        lo = limits_snapshot[joint].get('min')
        hi = limits_snapshot[joint].get('max')
        if lo is None or hi is None:
            return jsonify({'ok': False, 'msg': f'Falta MIN/MAX para {joint}'}), 400
        if float(lo) >= float(hi):
            return jsonify({'ok': False, 'msg': f'Rango inválido en {joint}: min>=max'}), 400

    cfg_path = Path(__file__).resolve().parent / 'servo_config.json'
    backup_path = cfg_path.with_name('servo_config.pre_calib.json')
    tmp_path = cfg_path.with_suffix('.json.tmp')

    try:
        with cfg_path.open('r', encoding='utf-8') as f:
            data = json.load(f)
        with backup_path.open('w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)

        joints = data.get('joints', {})
        for joint in CAL_JOINTS:
            if joint not in joints:
                return jsonify({'ok': False, 'msg': f'Joint no existe en servo_config: {joint}'}), 400
            joints[joint]['angle_safe_min_deg'] = float(limits_snapshot[joint]['min'])
            joints[joint]['angle_safe_max_deg'] = float(limits_snapshot[joint]['max'])
            home = float(joints[joint].get('angle_home_deg', 90.0))
            joints[joint]['angle_home_deg'] = max(
                float(limits_snapshot[joint]['min']),
                min(float(limits_snapshot[joint]['max']), home),
            )

        # Validación JSON + escritura atómica
        payload = json.dumps(data, ensure_ascii=False, indent=2)
        json.loads(payload)
        with tmp_path.open('w', encoding='utf-8') as f:
            f.write(payload)
        os.replace(tmp_path, cfg_path)
    except Exception as exc:
        return jsonify({'ok': False, 'msg': f'Error commit calibración: {exc}'}), 500

    _calibration_event('CAL_COMMIT', extra=f'backup={backup_path.name}')
    return jsonify({
        'ok': True,
        'msg': f'Calibración guardada. Backup: {backup_path.name}',
        'limits': limits_snapshot,
    })


@app.route('/api/calibration/state')
def api_calibration_state():
    with _calibration_lock:
        snap = json.loads(json.dumps(_calibration_state))
    return jsonify({'ok': True, 'calibration': snap})


@app.route('/api/calibrar_servos', methods=['POST'])
def api_calibrar_servos():
    if _is_calibration_enabled():
        return _calibration_blocked_response('/api/calibrar_servos')
    """Inicia la rutina de auto-calibracion de servos en un hilo."""
    global hilo_calibracion
    if hilo_calibracion and hilo_calibracion.is_alive():
        return jsonify({'ok': False, 'msg': 'Calibracion ya en progreso'})

    c = obtener_cerebro()

    def _callback(servo, fase, progreso):
        estado_calibracion['activo'] = fase != 'completado' or servo != 'sistema'
        estado_calibracion['servo'] = servo
        estado_calibracion['fase'] = fase
        estado_calibracion['progreso'] = progreso

    def _ejecutar():
        estado_calibracion['activo'] = True
        try:
            c.robot.calibrar_inicio(callback=_callback)
        except Exception as e:
            log.error(f"Error en calibracion: {e}")
        finally:
            estado_calibracion['activo'] = False

    hilo_calibracion = threading.Thread(target=_ejecutar, daemon=True)
    hilo_calibracion.start()
    return jsonify({'ok': True, 'msg': 'Calibracion de servos iniciada'})


@app.route('/api/calibrar_color', methods=['POST'])
def api_calibrar_color():
    if _is_calibration_enabled():
        return _calibration_blocked_response('/api/calibrar_color')
    """Calibra offsets HSV basandose en la iluminacion actual."""
    c = obtener_cerebro()
    if c.detector_color is None:
        return jsonify({'ok': False, 'msg': 'Detector de color no disponible'})

    img = c._capturar_imagen()
    if img is None:
        return jsonify({'ok': False, 'msg': 'No se pudo capturar imagen'})

    offsets = c.detector_color.calibrar_iluminacion(img)
    return jsonify({'ok': True, 'msg': 'Calibracion de color completada', 'offsets': offsets})


@app.route('/docs/puesta_en_marcha')
def docs_puesta_en_marcha():
    """Sirve PUESTA_EN_MARCHA.md del repositorio (texto plano legible en navegador)."""
    base = os.path.dirname(os.path.abspath(__file__))
    path = os.path.normpath(os.path.join(base, '..', 'PUESTA_EN_MARCHA.md'))
    if not os.path.isfile(path):
        return 'Documento PUESTA_EN_MARCHA.md no encontrado en la carpeta del proyecto.', 404
    with open(path, 'r', encoding='utf-8') as f:
        text = f.read()
    esc = html.escape(text)
    page = f"""<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Puesta en marcha</title>
<style>
body{{margin:0;background:#0f172a;color:#e2e8f0;font-family:system-ui,sans-serif}}
a.back{{color:#38bdf8;padding:16px 20px;display:inline-block}}
pre{{white-space:pre-wrap;word-break:break-word;font-family:ui-monospace,monospace;font-size:.78rem;
line-height:1.55;padding:8px 20px 40px;max-width:920px;margin:0 auto}}
</style>
</head>
<body>
<a class="back" href="/">&larr; Volver al panel</a>
<pre>{esc}</pre>
</body>
</html>"""
    return page


# ------------------------------------------------------------------ #
# HTML
# ------------------------------------------------------------------ #

HTML = """
<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Brazo Robotico Autonomo</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',system-ui,sans-serif;background:#0f172a;color:#e2e8f0;min-height:100vh}
.top-bar{background:linear-gradient(135deg,#1e3a5f,#0f172a);padding:12px 24px 14px;border-bottom:1px solid #334155}
.top-bar-row{display:flex;align-items:center;gap:16px;flex-wrap:wrap}
.top-bar h1{font-size:1.4rem;font-weight:700;background:linear-gradient(90deg,#38bdf8,#818cf8);-webkit-background-clip:text;-webkit-text-fill-color:transparent}
.top-bar-diag{margin-top:10px;padding-top:10px;border-top:1px solid #334155;display:flex;flex-wrap:wrap;align-items:center;gap:10px 16px;font-size:.78rem;color:#94a3b8}
.diag-pill{padding:2px 10px;border-radius:12px;font-weight:700;font-size:.72rem}
.diag-pill.ok{background:#14532d;color:#bbf7d0}
.diag-pill.bad{background:#7f1d1d;color:#fecaca}
.diag-motivo{font-family:ui-monospace,monospace;color:#cbd5e1;word-break:break-all;max-width:100%}
.cal-banner{display:none;margin-top:8px;padding:8px 12px;border:1px solid #ef4444;background:#7f1d1d;color:#fee2e2;border-radius:8px;font-size:.78rem;font-weight:700}
.voz-nota{margin-top:8px;font-size:.72rem;color:#a78bfa;line-height:1.4;display:none}
.voz-nota.warn{color:#fbbf24}
.estado-badge{padding:4px 14px;border-radius:20px;font-size:.75rem;font-weight:600;text-transform:uppercase;letter-spacing:.5px}
.estado-IDLE{background:#334155;color:#94a3b8}
.estado-ESCANEANDO{background:#0369a1;color:#e0f2fe}
.estado-VISION_FALLA{background:#b91c1c;color:#fee2e2}
.estado-PLANIFICANDO{background:#7c3aed;color:#ede9fe}
.estado-RECOGIENDO{background:#ea580c;color:#fff7ed}
.estado-TRANSPORTANDO{background:#0891b2;color:#ecfeff}
.estado-DEPOSITANDO{background:#16a34a;color:#f0fdf4}
.estado-RECUPERANDO_ERROR{background:#dc2626;color:#fef2f2}
.estado-PAUSADO{background:#ca8a04;color:#fefce8}
.estado-COMPLETADO{background:#059669;color:#ecfdf5}
.main{display:grid;grid-template-columns:1fr 380px;gap:16px;padding:16px;max-width:1400px;margin:0 auto}
@media(max-width:900px){.main{grid-template-columns:1fr}}
.video-panel{background:#1e293b;border-radius:12px;overflow:hidden;border:1px solid #334155}
.video-panel img{width:100%;display:block;min-height:300px;background:#000}
.side{display:flex;flex-direction:column;gap:12px}
.card{background:#1e293b;border-radius:12px;padding:16px;border:1px solid #334155}
.card h3{font-size:.85rem;text-transform:uppercase;letter-spacing:.8px;color:#64748b;margin-bottom:10px}
.btn-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.btn{padding:10px;border:none;border-radius:8px;font-weight:600;font-size:.85rem;cursor:pointer;transition:all .2s}
.btn:active{transform:scale(.96)}
.btn-start{background:linear-gradient(135deg,#22c55e,#16a34a);color:#fff}
.btn-pause{background:linear-gradient(135deg,#f59e0b,#d97706);color:#fff}
.btn-resume{background:linear-gradient(135deg,#3b82f6,#2563eb);color:#fff}
.btn-stop{background:linear-gradient(135deg,#ef4444,#dc2626);color:#fff}
.btn-home{background:linear-gradient(135deg,#8b5cf6,#7c3aed);color:#fff}
.btn-scan{background:linear-gradient(135deg,#06b6d4,#0891b2);color:#fff}
.btn-calib{background:linear-gradient(135deg,#f97316,#ea580c);color:#fff}
.btn-calib-color{background:linear-gradient(135deg,#a855f7,#7c3aed);color:#fff}
.btn-emergency{background:#dc2626;color:#fff;grid-column:1/-1;font-size:1rem;padding:14px;animation:pulse 2s infinite}
@keyframes pulse{0%,100%{box-shadow:0 0 0 0 rgba(220,38,38,.4)}50%{box-shadow:0 0 0 8px rgba(220,38,38,0)}}
.stat-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.stat{background:#0f172a;border-radius:8px;padding:10px;text-align:center}
.stat .val{font-size:1.5rem;font-weight:700;color:#38bdf8}
.stat .lbl{font-size:.7rem;color:#64748b;margin-top:2px}
.obj-list,.rec-list{max-height:150px;overflow-y:auto;font-size:.8rem}
.obj-item,.rec-item{padding:6px 8px;border-radius:6px;margin-bottom:4px;display:flex;justify-content:space-between;align-items:center}
.obj-item{background:#0f172a}
.rec-item{background:#0f172a}
.color-dot{width:12px;height:12px;border-radius:50%;display:inline-block;margin-right:6px}
.dot-rojo{background:#ef4444}.dot-azul{background:#3b82f6}.dot-verde{background:#22c55e}.dot-amarillo{background:#eab308}
.dot-naranja{background:#f97316}.dot-morado{background:#a855f7}.dot-desconocido{background:#6b7280}
.log-box{max-height:120px;overflow-y:auto;font-size:.75rem;background:#0f172a;border-radius:8px;padding:8px;font-family:monospace;color:#94a3b8}
.manual-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:6px}
.btn-sm{padding:8px 4px;font-size:.75rem;border:none;border-radius:6px;cursor:pointer;background:#334155;color:#e2e8f0;font-weight:600}
.btn-sm:hover{background:#475569}
.slider-wrap{display:grid;gap:8px;margin-top:8px}
.slider-row{display:grid;grid-template-columns:72px 1fr 56px;align-items:center;gap:8px}
.slider-row label{font-size:.75rem;color:#cbd5e1;font-weight:600}
.slider-row input[type=range]{width:100%}
.slider-val{font-size:.74rem;color:#38bdf8;font-weight:700;text-align:right}
.pos-bar-container{margin-top:4px}
.pos-row{display:flex;align-items:center;gap:8px;margin-bottom:6px;font-size:.75rem}
.pos-label{width:65px;text-align:right;color:#94a3b8;font-weight:600}
.pos-track{flex:1;height:8px;background:#0f172a;border-radius:4px;position:relative;overflow:hidden}
.pos-fill{height:100%;border-radius:4px;transition:width .3s;background:linear-gradient(90deg,#38bdf8,#818cf8)}
.pos-val{width:35px;color:#38bdf8;font-weight:700;font-size:.7rem}
.calib-status{font-size:.75rem;color:#f59e0b;margin-top:8px;min-height:18px}
.progress-bar{height:6px;background:#0f172a;border-radius:3px;margin-top:4px;overflow:hidden}
.progress-fill{height:100%;background:linear-gradient(90deg,#f97316,#22c55e);transition:width .5s;border-radius:3px}
.checklist-wrap{max-width:1400px;margin:0 auto;padding:0 16px 28px}
.checklist-card h3{margin-bottom:4px}
.checklist-top{display:flex;flex-wrap:wrap;align-items:center;justify-content:space-between;gap:10px;margin-bottom:12px}
.checklist-progress{font-size:.8rem;color:#38bdf8;font-weight:600}
.checklist-actions{display:flex;flex-wrap:wrap;gap:8px}
.btn-link,.btn-ghost{background:transparent;border:1px solid #475569;color:#94a3b8;padding:6px 12px;border-radius:8px;font-size:.72rem;cursor:pointer;text-decoration:none;font-weight:600}
.btn-link:hover,.btn-ghost:hover{color:#e2e8f0;border-color:#64748b}
.checklist-group{border-bottom:1px solid #334155;padding-bottom:12px;margin-bottom:12px}
.checklist-group:last-child{border-bottom:none;margin-bottom:0;padding-bottom:0}
.checklist-group h4{font-size:.72rem;text-transform:uppercase;letter-spacing:.6px;color:#64748b;margin-bottom:8px}
.checklist-row{display:flex;align-items:flex-start;gap:10px;font-size:.78rem;margin-bottom:7px;color:#cbd5e1;line-height:1.4}
.checklist-row input[type=checkbox]{margin-top:3px;flex-shrink:0;width:16px;height:16px;accent-color:#38bdf8;cursor:pointer}
</style>
</head>
<body>

<div class="top-bar">
  <div class="top-bar-row">
    <h1>Brazo Robotico Autonomo</h1>
    <span class="estado-badge estado-IDLE" id="badge-estado">IDLE</span>
  </div>
  <div class="top-bar-diag">
    <span>Ultimo escaneo: <span class="diag-pill ok" id="diag-escaneo-ok">—</span></span>
    <span class="diag-motivo" id="diag-escaneo-motivo" title="Motivo interno del ultimo escaneo">—</span>
  </div>
  <div class="cal-banner" id="cal-banner">MODO CALIBRACIÓN ACTIVO — Controles legacy y autonomía bloqueados.</div>
  <p class="voz-nota" id="voz-nota-ok">Voz habilitada en configuracion: reconocimiento por <strong>Google</strong> (requiere <strong>Internet</strong>) y microfono USB bien configurado. Ver HARDWARE_AUDIO.md.</p>
  <p class="voz-nota warn" id="voz-nota-warn" style="display:none">Voz activada en config pero el asistente no arranco: revisa <code>pip install -r requirements-voice.txt</code> y el microfono.</p>
</div>

<div class="main">
  <div>
    <div class="video-panel">
      <img id="video" src="/video_feed" alt="Video en vivo">
    </div>
    <div class="card" style="margin-top:12px" id="legacy-manual-card">
      <h3>Control Manual Rapido</h3>
      <div class="manual-grid">
        <button class="btn-sm" onclick="manualMove('shoulder',1)">Hombro +</button>
        <button class="btn-sm" onclick="manualMove('elbow',1)">Codo +</button>
        <button class="btn-sm" onclick="manualMove('wrist',1)">Muneca +</button>
        <button class="btn-sm" onclick="manualMove('shoulder',-1)">Hombro -</button>
        <button class="btn-sm" onclick="manualMove('elbow',-1)">Codo -</button>
        <button class="btn-sm" onclick="manualMove('wrist',-1)">Muneca -</button>
        <button class="btn-sm" style="background:#166534" onclick="manualMove('gripper',1)">Pinza Abrir</button>
        <button class="btn-sm" style="background:#475569" onclick="apiPost('/api/home')">HOME</button>
        <button class="btn-sm" style="background:#991b1b" onclick="manualMove('gripper',-1)">Pinza Cerrar</button>
        <button class="btn-sm" style="background:#0e7490" onclick="moverBase(-1)">Base Izq</button>
        <button class="btn-sm" style="background:#64748b">---</button>
        <button class="btn-sm" style="background:#0e7490" onclick="moverBase(1)">Base Der</button>
      </div>
    </div>
    <div class="card" style="margin-top:12px" id="legacy-slider-card">
      <h3>Modo Calibracion Segura</h3>
      <div style="display:flex;gap:8px;flex-wrap:wrap;margin-bottom:8px">
        <button class="btn-sm" style="background:#065f46" onclick="apiPost('/api/calibration/enable')">Calibracion ON</button>
        <button class="btn-sm" style="background:#334155" onclick="apiPost('/api/calibration/disable')">Calibracion OFF</button>
        <button class="btn-sm" style="background:#991b1b" onclick="apiPost('/api/calibration/stop')">STOP</button>
        <button class="btn-sm" style="background:#b91c1c" onclick="apiPost('/api/calibration/emergency')">EMERGENCY</button>
      </div>
      <div style="display:flex;gap:8px;flex-wrap:wrap;margin-bottom:8px">
        <button class="btn-sm" onclick="calibSelect('base')">Joint: Base</button>
        <button class="btn-sm" onclick="calibSelect('shoulder')">Joint: Hombro</button>
        <button class="btn-sm" onclick="calibSelect('elbow')">Joint: Codo</button>
        <button class="btn-sm" onclick="calibSelect('wrist')">Joint: Muneca</button>
      </div>
      <div style="display:flex;gap:8px;flex-wrap:wrap;margin-bottom:8px">
        <button class="btn-sm" style="background:#0f766e" onclick="calibStep(-1)">-1°</button>
        <button class="btn-sm" style="background:#0f766e" onclick="calibStep(1)">+1°</button>
        <button class="btn-sm" style="background:#a16207" onclick="apiPost('/api/calibration/save_min')">Guardar MIN</button>
        <button class="btn-sm" style="background:#a16207" onclick="apiPost('/api/calibration/save_max')">Guardar MAX</button>
        <button class="btn-sm" style="background:#7c3aed" onclick="apiPost('/api/calibration/commit')">Commit Limites</button>
      </div>
      <div style="font-size:.76rem;color:#cbd5e1;line-height:1.45">
        <div>Estado: <strong id="cal-mode">OFF</strong> · Runtime: <strong id="cal-runtime">IDLE</strong></div>
        <div>Joint activo: <strong id="cal-joint">shoulder</strong></div>
      </div>
    </div>
    <div class="card" style="margin-top:12px">
      <h3>Control por Barras (Tiempo Real)</h3>
      <div class="slider-wrap">
        <div class="slider-row">
          <label for="sl-base">Base</label>
          <input id="sl-base" type="range" min="0" max="180" step="1" value="90"
                 oninput="sliderMoveJoint('base', this.value)">
          <span class="slider-val" id="slv-base">90°</span>
        </div>
        <div class="slider-row">
          <label for="sl-shoulder">Hombro</label>
          <input id="sl-shoulder" type="range" min="15" max="165" step="1" value="90"
                 oninput="sliderMoveJoint('shoulder', this.value)">
          <span class="slider-val" id="slv-shoulder">90°</span>
        </div>
        <div class="slider-row">
          <label for="sl-elbow">Codo</label>
          <input id="sl-elbow" type="range" min="20" max="160" step="1" value="90"
                 oninput="sliderMoveJoint('elbow', this.value)">
          <span class="slider-val" id="slv-elbow">90°</span>
        </div>
        <div class="slider-row">
          <label for="sl-wrist">Muneca</label>
          <input id="sl-wrist" type="range" min="20" max="170" step="1" value="90"
                 oninput="sliderMoveJoint('wrist', this.value)">
          <span class="slider-val" id="slv-wrist">90°</span>
        </div>
        <div class="slider-row">
          <label for="sl-gripper">Pinza 360</label>
          <input id="sl-gripper" type="range" min="-100" max="100" step="1" value="0"
                 oninput="sliderMoveGripper(this.value)" onchange="sliderStopGripper()">
          <span class="slider-val" id="slv-gripper">0%</span>
        </div>
      </div>
    </div>
    <div class="card" style="margin-top:12px">
      <h3>Posicion Estimada</h3>
      <div class="pos-bar-container" id="pos-bars">
        <div class="pos-row"><span class="pos-label">Hombro</span><div class="pos-track"><div class="pos-fill" id="pos-shoulder" style="width:50%"></div></div><span class="pos-val" id="pv-shoulder">50%</span></div>
        <div class="pos-row"><span class="pos-label">Codo</span><div class="pos-track"><div class="pos-fill" id="pos-elbow" style="width:50%"></div></div><span class="pos-val" id="pv-elbow">50%</span></div>
        <div class="pos-row"><span class="pos-label">Muneca</span><div class="pos-track"><div class="pos-fill" id="pos-wrist" style="width:50%"></div></div><span class="pos-val" id="pv-wrist">50%</span></div>
        <div class="pos-row"><span class="pos-label">Pinza</span><div class="pos-track"><div class="pos-fill" id="pos-gripper" style="width:50%"></div></div><span class="pos-val" id="pv-gripper">50%</span></div>
      </div>
    </div>
  </div>

  <div class="side">
    <div class="card" id="legacy-autonomy-card">
      <h3>Controles</h3>
      <div class="btn-grid">
        <button class="btn btn-start" onclick="apiPost('/api/iniciar')">Iniciar</button>
        <button class="btn btn-pause" onclick="apiPost('/api/pausar')">Pausar</button>
        <button class="btn btn-resume" onclick="apiPost('/api/reanudar')">Reanudar</button>
        <button class="btn btn-stop" onclick="apiPost('/api/detener')">Detener</button>
        <button class="btn btn-home" onclick="apiPost('/api/home')">Home</button>
        <button class="btn btn-scan" onclick="apiPost('/api/escanear')">Escanear</button>
        <button class="btn btn-calib" onclick="apiPost('/api/calibrar_servos')">Calibrar Servos</button>
        <button class="btn btn-calib-color" onclick="apiPost('/api/calibrar_color')">Calibrar Color</button>
        <button class="btn btn-emergency" onclick="apiPost('/api/emergencia')">EMERGENCIA</button>
        <button class="btn btn-reset-emerg" id="btn-reset-emerg" style="display:none;grid-column:1/-1;background:#7c3aed;color:#fff;padding:10px;font-size:.85rem;font-weight:600;border:none;border-radius:8px;cursor:pointer" onclick="resetEmergencia()">Reset Emergencia</button>
      </div>
      <div class="safe-status" id="safe-status" style="font-size:.72rem;margin-top:6px;min-height:16px"></div>
      <div class="calib-status" id="calib-status"></div>
      <div class="progress-bar" id="calib-progress-bar" style="display:none"><div class="progress-fill" id="calib-progress-fill" style="width:0%"></div></div>
    </div>

    <div class="card">
      <h3>Estadisticas</h3>
      <div class="stat-grid">
        <div class="stat"><div class="val" id="st-detectados">0</div><div class="lbl">Detectados</div></div>
        <div class="stat"><div class="val" id="st-exitos">0</div><div class="lbl">Agarres OK</div></div>
        <div class="stat"><div class="val" id="st-fallos">0</div><div class="lbl">Fallos</div></div>
        <div class="stat"><div class="val" id="st-depositos">0</div><div class="lbl">Depositos</div></div>
        <div class="stat"><div class="val" id="st-recuperados">0</div><div class="lbl">Errores Recup.</div></div>
        <div class="stat"><div class="val" id="st-ciclos">0</div><div class="lbl">Ciclos</div></div>
      </div>
    </div>

    <div class="card">
      <h3>Objetos Detectados</h3>
      <div class="obj-list" id="obj-list"><em style="color:#64748b">Sin datos</em></div>
    </div>

    <div class="card">
      <h3>Recipientes</h3>
      <div class="rec-list" id="rec-list"><em style="color:#64748b">Sin datos</em></div>
    </div>

    <div class="card">
      <h3>Historial</h3>
      <div class="log-box" id="log-box">Esperando eventos...</div>
    </div>
  </div>
</div>

<div class="checklist-wrap">
  <div class="card checklist-card">
    <div class="checklist-top">
      <h3 style="margin:0">Checklist puesta en marcha</h3>
      <span class="checklist-progress" id="checklist-progress">0/0 completados</span>
    </div>
    <p style="font-size:.75rem;color:#64748b;margin-bottom:12px;line-height:1.45">
      Lo que el repositorio no puede cerrar sin tu mesa: calibración, YOLO acorde a tus piezas, seguridad y audio.
      Las casillas se guardan en <strong>este navegador</strong> (localStorage).
    </p>
    <div class="checklist-actions" style="margin-bottom:14px">
      <a class="btn-link" href="/docs/puesta_en_marcha" target="_blank" rel="noopener">Guía completa (markdown)</a>
      <button type="button" class="btn-ghost" id="checklist-reset">Restablecer casillas</button>
    </div>
    <div class="checklist-group">
      <h4>1 · Calibración servos y color</h4>
      <label class="checklist-row"><input type="checkbox" data-id="c1-1"> Revisar <code>servo_config_legacy.json</code> (tipo servo, pulsos min/máx).</label>
      <label class="checklist-row"><input type="checkbox" data-id="c1-2"> Calibrar servos desde la web (topes reales).</label>
      <label class="checklist-row"><input type="checkbox" data-id="c1-3"> Probar HOME y manual: sin forzar mecánica ni vibración excesiva.</label>
      <label class="checklist-row"><input type="checkbox" data-id="c1-4"> Calibrar color con la misma luz que usarás en producción.</label>
      <label class="checklist-row"><input type="checkbox" data-id="c1-5"> Escanear con recipientes fijos: lista de recipientes estable.</label>
      <label class="checklist-row"><input type="checkbox" data-id="c1-6"> Con brazo real: <code>PERMITIR_DETECCION_SIMULADA=False</code> (no inventar objetos si falla la visión).</label>
    </div>
    <div class="checklist-group">
      <h4>2 · YOLO y clases</h4>
      <label class="checklist-row"><input type="checkbox" data-id="c2-1"> Saber qué clases predice el modelo actual (p. ej. COCO).</label>
      <label class="checklist-row"><input type="checkbox" data-id="c2-2"> Si tus objetos son otros: plan de modelo propio / LAB_WORKBENCH.</label>
      <label class="checklist-row"><input type="checkbox" data-id="c2-3"> Rellenar <code>YOLO_LAB_CLASE_A_COLOR</code> si usas clases propias.</label>
      <label class="checklist-row"><input type="checkbox" data-id="c2-4"> Ajustar <code>CONFIANZA_MINIMA_DETECCION</code> según falsos positivos.</label>
      <label class="checklist-row"><input type="checkbox" data-id="c2-5"> Valorar subir <code>imgsz</code> si las piezas son muy pequeñas (más carga CPU).</label>
    </div>
    <div class="checklist-group">
      <h4>3 · Mecánica y seguridad</h4>
      <label class="checklist-row"><input type="checkbox" data-id="c3-1"> Fuentes adecuadas (servos, incluida base MG996R) y masa común correcta.</label>
      <label class="checklist-row"><input type="checkbox" data-id="c3-2"> Zona de trabajo despejada (personas y cables fuera de alcance).</label>
      <label class="checklist-row"><input type="checkbox" data-id="c3-3"> Probar parada de emergencia y conocer el comportamiento.</label>
      <label class="checklist-row"><input type="checkbox" data-id="c3-4"> Primera sesión con velocidad autónoma conservadora.</label>
      <label class="checklist-row"><input type="checkbox" data-id="c3-5"> Revisar cableado I2C, base en canal 4 y CSI (CONEXIONES.md).</label>
    </div>
    <div class="checklist-group">
      <h4>4 · Audio (si usas voz)</h4>
      <label class="checklist-row"><input type="checkbox" data-id="c4-1"> Pruebas <code>arecord</code> / <code>aplay</code> / <code>espeak-ng</code> (HARDWARE_AUDIO.md).</label>
      <label class="checklist-row"><input type="checkbox" data-id="c4-2"> Si hay varios micrófonos: fijar <code>VOZ_MIC_DEVICE_INDEX</code>.</label>
      <label class="checklist-row"><input type="checkbox" data-id="c4-3"> Internet estable si usas reconocimiento Google.</label>
      <label class="checklist-row"><input type="checkbox" data-id="c4-4"> <code>requirements-voice.txt</code> + paquetes del sistema instalados.</label>
    </div>
    <div class="checklist-group">
      <h4>5 · Orden sugerido (primer día)</h4>
      <label class="checklist-row"><input type="checkbox" data-id="c5-1"> Cableado y alimentación antes de movimiento autónomo.</label>
      <label class="checklist-row"><input type="checkbox" data-id="c5-2"> Web, vídeo y control manual antes de modo autónomo.</label>
      <label class="checklist-row"><input type="checkbox" data-id="c5-3"> Calibrar servos y color antes de confiar en pick &amp; place.</label>
      <label class="checklist-row"><input type="checkbox" data-id="c5-4"> Validar detección con objetos que el modelo sí conozca.</label>
      <label class="checklist-row"><input type="checkbox" data-id="c5-5"> Audio probado antes de depurar comandos de voz.</label>
    </div>
  </div>
</div>

<script>
let CAL_MODE_ACTIVE = false;
const CHECKLIST_STORAGE_KEY = 'brazo_checklist_v1';
function checklistLoad(){
  try { return JSON.parse(localStorage.getItem(CHECKLIST_STORAGE_KEY) || '{}'); } catch(e){ return {}; }
}
function checklistSave(state){
  localStorage.setItem(CHECKLIST_STORAGE_KEY, JSON.stringify(state));
}
function checklistUpdateProgress(){
  const boxes = document.querySelectorAll('.checklist-row input[type=checkbox]');
  let done = 0, total = 0;
  boxes.forEach(b => { total++; if (b.checked) done++; });
  const el = document.getElementById('checklist-progress');
  if (el) el.textContent = done + '/' + total + ' completados';
}
function checklistInit(){
  const state = checklistLoad();
  document.querySelectorAll('.checklist-row input[type=checkbox]').forEach(cb => {
    if (state[cb.dataset.id]) cb.checked = true;
    cb.addEventListener('change', () => {
      const s = checklistLoad();
      s[cb.dataset.id] = cb.checked;
      checklistSave(s);
      checklistUpdateProgress();
    });
  });
  checklistUpdateProgress();
  const reset = document.getElementById('checklist-reset');
  if (reset) reset.addEventListener('click', () => {
    if (!confirm('Borrar el progreso del checklist en este navegador?')) return;
    localStorage.removeItem(CHECKLIST_STORAGE_KEY);
    document.querySelectorAll('.checklist-row input[type=checkbox]').forEach(cb => { cb.checked = false; });
    checklistUpdateProgress();
  });
}
document.addEventListener('DOMContentLoaded', checklistInit);

function apiPost(url, body){
  fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:body?JSON.stringify(body):'{}'})
  .then(r=>r.json()).then(d=>{if(d.msg)console.log(d.msg)}).catch(e=>console.error(e));
}
function manualMove(joint,dir){
  if(CAL_MODE_ACTIVE) return;
  apiPost('/api/mover',{joint,dir});
}
function moverBase(dir){
  if(CAL_MODE_ACTIVE) return;
  apiPost('/api/mover',{joint:'base',dir});
}
function calibSelect(joint){
  apiPost('/api/calibration/select_joint',{joint});
}
function calibStep(dir){
  apiPost('/api/calibration/step',{dir});
}
const _sliderTimers = {};
function _debounceByKey(key, fn, ms){
  if(_sliderTimers[key]) clearTimeout(_sliderTimers[key]);
  _sliderTimers[key]=setTimeout(fn, ms);
}
function sliderMoveJoint(joint, value){
  if(CAL_MODE_ACTIVE) return;
  const v=Math.round(Number(value)||0);
  const lbl=document.getElementById('slv-'+joint);
  if(lbl) lbl.textContent=v+'°';
  _debounceByKey('joint-'+joint, ()=>apiPost('/api/set_angle',{joint,angle:v}), 70);
}
function sliderMoveGripper(value){
  if(CAL_MODE_ACTIVE) return;
  const v=Math.round(Number(value)||0);
  const lbl=document.getElementById('slv-gripper');
  if(lbl) lbl.textContent=v+'%';
  _debounceByKey('gripper-speed', ()=>apiPost('/api/gripper_continuo',{speed:v}), 70);
}
function sliderStopGripper(){
  if(CAL_MODE_ACTIVE) return;
  const sl=document.getElementById('sl-gripper');
  const lbl=document.getElementById('slv-gripper');
  if(sl) sl.value = 0;
  if(lbl) lbl.textContent='0%';
  apiPost('/api/gripper_continuo',{speed:0});
}
function setLegacyUiDisabled(disabled){
  const banner=document.getElementById('cal-banner');
  if(banner) banner.style.display = disabled ? 'block' : 'none';

  // Bloquear controles legacy para evitar confusión operacional.
  document.querySelectorAll('#legacy-manual-card button').forEach(el=>{
    el.disabled = disabled;
    el.style.opacity = disabled ? '0.45' : '1';
    el.style.pointerEvents = disabled ? 'none' : 'auto';
  });
  document.querySelectorAll('#legacy-slider-card input, #legacy-slider-card button').forEach(el=>{
    el.disabled = disabled;
    el.style.opacity = disabled ? '0.45' : '1';
    el.style.pointerEvents = disabled ? 'none' : 'auto';
  });
  // Controles autónomos/macros legacy bloqueados en calibración.
  document.querySelectorAll('#legacy-autonomy-card .btn-grid .btn').forEach(el=>{
    el.disabled = disabled;
    el.style.opacity = disabled ? '0.45' : '1';
    el.style.pointerEvents = disabled ? 'none' : 'auto';
  });
}
function resetEmergencia(){
  if(!confirm('¿Confirmas que el brazo está en posición segura y es seguro reanudar?'))return;
  fetch('/api/reset_emergency',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'})
  .then(r=>r.json()).then(d=>{console.log(d.msg)}).catch(e=>console.error(e));
}

function actualizarUI(){
  fetch('/api/estado').then(r=>r.json()).then(d=>{
    const badge=document.getElementById('badge-estado');
    badge.textContent=d.estado;
    badge.className='estado-badge estado-'+d.estado;

    const pill=document.getElementById('diag-escaneo-ok');
    const mot=document.getElementById('diag-escaneo-motivo');
    if(pill && typeof d.ultimo_escaneo_ok==='boolean'){
      pill.textContent=d.ultimo_escaneo_ok?'OK':'Falla';
      pill.className='diag-pill '+(d.ultimo_escaneo_ok?'ok':'bad');
    }
    if(mot && d.ultimo_escaneo_motivo!==undefined){
      mot.textContent=d.ultimo_escaneo_motivo;
    }

    const vozOk=document.getElementById('voz-nota-ok');
    const vozWarn=document.getElementById('voz-nota-warn');
    if(vozOk && vozWarn){
      const vcfg=!!d.voz_config_habilitada;
      const vrun=!!d.voz_activa;
      if(!vcfg){
        vozOk.style.display='none';
        vozWarn.style.display='none';
      } else if(vrun){
        vozOk.style.display='block';
        vozWarn.style.display='none';
      } else {
        vozOk.style.display='none';
        vozWarn.style.display='block';
      }
    }

    const s=d.estadisticas;
    document.getElementById('st-detectados').textContent=s.objetos_detectados;
    document.getElementById('st-exitos').textContent=s.agarres_exitosos;
    document.getElementById('st-fallos').textContent=s.agarres_fallidos;
    document.getElementById('st-depositos').textContent=s.depositos_exitosos;
    document.getElementById('st-recuperados').textContent=s.errores_recuperados;
    document.getElementById('st-ciclos').textContent=s.ciclos_completados;

    if(d.posiciones){
      const servoRangeDeg={shoulder:270,elbow:270,wrist:180,gripper:360};
      ['shoulder','elbow','wrist','gripper'].forEach(j=>{
        const v=d.posiciones[j];
        if(v!==undefined){
          const pct=Math.round(v*100);
          const deg=Math.round(v*(servoRangeDeg[j]||180));
          const bar=document.getElementById('pos-'+j);
          const lbl=document.getElementById('pv-'+j);
          if(bar)bar.style.width=pct+'%';
          if(lbl)lbl.textContent=(j==='gripper')?(deg+'° (cont)'):(deg+'°');
        }
      });
    }
    if(d.safe_angles){
      ['base','shoulder','elbow','wrist'].forEach(j=>{
        const av=d.safe_angles[j];
        if(av===undefined) return;
        const slider=document.getElementById('sl-'+j);
        const lbl=document.getElementById('slv-'+j);
        if(slider && document.activeElement !== slider){
          const sv=Math.round(av);
          slider.value=sv;
          if(lbl) lbl.textContent=sv+'°';
        }
      });
    }
    const calMode=document.getElementById('cal-mode');
    const calRun=document.getElementById('cal-runtime');
    const calJoint=document.getElementById('cal-joint');
    CAL_MODE_ACTIVE = !!d.calibration_mode;
    setLegacyUiDisabled(CAL_MODE_ACTIVE);
    if(calMode) calMode.textContent=d.calibration_mode?'ON':'OFF';
    if(calRun) calRun.textContent=d.calibration_runtime||'IDLE';
    if(calJoint) calJoint.textContent=d.calibration_active_joint||'-';

    if(d.calibracion && d.calibracion.activo){
      document.getElementById('calib-status').textContent='Calibrando: '+d.calibracion.servo+' ('+d.calibracion.fase+')';
      document.getElementById('calib-progress-bar').style.display='block';
      document.getElementById('calib-progress-fill').style.width=d.calibracion.progreso+'%';
    } else {
      document.getElementById('calib-status').textContent='';
      document.getElementById('calib-progress-bar').style.display='none';
    }

    const ol=document.getElementById('obj-list');
    if(d.objetos.length){
      ol.innerHTML=d.objetos.map(o=>`<div class="obj-item"><span><span class="color-dot dot-${o.color}"></span>${o.clase}</span><span>${(o.confianza*100).toFixed(0)}%</span></div>`).join('');
    } else {ol.innerHTML='<em style="color:#64748b">Ninguno</em>'}

    const rl=document.getElementById('rec-list');
    if(d.recipientes.length){
      rl.innerHTML=d.recipientes.map(r=>`<div class="rec-item"><span><span class="color-dot dot-${r.color}"></span>${r.color}</span><span>${r.depositados} obj</span></div>`).join('');
    } else {rl.innerHTML='<em style="color:#64748b">Ninguno</em>'}

    const lb=document.getElementById('log-box');
    if(d.historial_reciente.length){
      lb.innerHTML=d.historial_reciente.slice(-8).reverse().map(h=>`<div>${h.timestamp.split('T')[1].split('.')[0]} [${h.tipo}] ${JSON.stringify(h.datos).substring(0,80)}</div>`).join('');
    }

    // Estado SafeController
    const safeStatus=document.getElementById('safe-status');
    const btnReset=document.getElementById('btn-reset-emerg');
    if(safeStatus){
      if(d.safe_emergency){
        safeStatus.innerHTML='<span style="color:#ef4444;font-weight:700">⚡ SAFE: Emergency Stop activo</span>';
        if(btnReset)btnReset.style.display='block';
      } else {
        const simTag=d.safe_simulation?' <span style="color:#f59e0b">[SIM]</span>':'';
        safeStatus.innerHTML='<span style="color:#22c55e">&#10003; SAFE: OK</span>'+simTag;
        if(btnReset)btnReset.style.display='none';
      }
    }
  }).catch(()=>{});
}

setInterval(actualizarUI,1500);
actualizarUI();
</script>
</body>
</html>
"""


@app.route('/')
def index():
    return render_template_string(HTML)


if __name__ == '__main__':
    print("=" * 60)
    print("  BRAZO ROBOTICO AUTONOMO - Interfaz Web")
    print("  Abrir en navegador: http://<IP_RASPBERRY>:5000")
    print("=" * 60)
    _registrar_voz_si_habilitada()
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
