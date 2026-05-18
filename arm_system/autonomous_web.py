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

try:
    import psutil
except Exception:
    psutil = None

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

_learning_state = {
    'enabled': False,
    'events': [],
}
_learning_dir = Path(__file__).resolve().parent.parent / 'learning_records'
_learning_dir.mkdir(parents=True, exist_ok=True)

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


def _register_learning_event(event_type: str, payload: dict) -> None:
    if not _learning_state['enabled']:
        return
    _learning_state['events'].append({
        'ts': datetime.utcnow().isoformat() + 'Z',
        'type': event_type,
        'payload': payload,
    })
    log.info('LEARNING EVENT %s %s', event_type, payload)


def _is_calibration_enabled() -> bool:
    with _calibration_lock:
        return bool(_calibration_state['enabled'])


def _calibration_blocked_response(endpoint_name: str):
    return jsonify({
        'ok': False,
        'msg': f'Calibración segura activa. Endpoint bloqueado: {endpoint_name}'
    }), 423


def _obtener_metricas_sistema():
    metrics = {
        'cpu_percent': None,
        'ram_percent': None,
        'ram_total_mb': None,
        'ram_used_mb': None,
        'temperature_celsius': None,
        'platform': sys.platform,
    }
    if psutil:
        try:
            metrics['cpu_percent'] = psutil.cpu_percent(interval=None)
        except Exception:
            pass
        try:
            mem = psutil.virtual_memory()
            metrics['ram_percent'] = mem.percent
            metrics['ram_total_mb'] = int(mem.total / 1024 / 1024)
            metrics['ram_used_mb'] = int((mem.total - mem.available) / 1024 / 1024)
        except Exception:
            pass
        try:
            temps = psutil.sensors_temperatures() if hasattr(psutil, 'sensors_temperatures') else {}
            if temps:
                first = next(iter(temps.values()), None)
                if first:
                    metrics['temperature_celsius'] = float(first[0].current)
        except Exception:
            pass
    else:
        if os.path.exists('/sys/class/thermal/thermal_zone0/temp'):
            try:
                with open('/sys/class/thermal/thermal_zone0/temp', 'r', encoding='utf-8') as f:
                    raw = f.read().strip()
                metrics['temperature_celsius'] = round(int(raw) / 1000.0, 1)
            except Exception:
                pass
        if os.path.exists('/proc/meminfo'):
            try:
                with open('/proc/meminfo', 'r', encoding='utf-8') as f:
                    lines = f.readlines()
                meminfo = {line.split(':')[0]: int(line.split(':')[1].strip().split()[0]) for line in lines if ':' in line}
                total = meminfo.get('MemTotal')
                free = meminfo.get('MemFree')
                available = meminfo.get('MemAvailable', free)
                if total and available is not None:
                    metrics['ram_total_mb'] = int(total / 1024)
                    metrics['ram_used_mb'] = int((total - available) / 1024)
                    metrics['ram_percent'] = round(100.0 * (total - available) / total, 1)
            except Exception:
                pass
    return metrics


def _diagnosticar_sistema(c):
    camera_disabled = getattr(c, '_camera_disabled_runtime', False)
    vision_disabled = getattr(c, '_vision_disabled_runtime', False)
    camera_present = c.camara is not None
    yolo_present = c.detector_yolo is not None
    color_present = c.detector_color is not None

    diagnostics = []

    if camera_disabled:
        diagnostics.append({
            'title': 'Cámara deshabilitada',
            'status': 'warn',
            'category': 'Cámara',
            'description': 'La captura de video está desactivada en tiempo de ejecución o por configuración.',
            'why': 'BR_DISABLE_CAMERA=1 o CAMARA_HABILITADA=False.',
            'fix': 'Verifique la configuración y habilite la cámara si necesita visión en tiempo real.',
        })
        camera_status = {'level': 'warn', 'message': 'Cámara desactivada por configuración'}
    elif not camera_present:
        diagnostics.append({
            'title': 'Cámara no encontrada',
            'status': 'error',
            'category': 'Cámara',
            'description': 'El sistema no pudo inicializar la cámara de captura.',
            'why': 'Puede faltar hardware, cable CSI suelto o librería de cámara no disponible.',
            'fix': 'Compruebe el cable, habilite la cámara en raspi-config o instale los controladores necesarios.',
        })
        camera_status = {'level': 'error', 'message': 'Cámara no inicializada'}
    else:
        diagnostics.append({
            'title': 'Cámara operativa',
            'status': 'ok',
            'category': 'Cámara',
            'description': 'El flujo de video está disponible para supervisión y detección.',
            'why': 'La cámara respondió correctamente al inicializarse.',
            'fix': 'Ninguno.',
        })
        camera_status = {'level': 'ok', 'message': 'Cámara operativa'}

    if vision_disabled:
        diagnostics.append({
            'title': 'Visión desactivada',
            'status': 'warn',
            'category': 'Visión',
            'description': 'El stack de visión (YOLO + color) está deshabilitado en tiempo de ejecución.',
            'why': 'BR_DISABLE_VISION=1.',
            'fix': 'Habilite la visión si desea usar detección visual.',
        })
        vision_status = {'level': 'warn', 'message': 'Visión desactivada por configuración'}
    else:
        missing = []
        if not yolo_present:
            missing.append('YOLO')
        if not color_present:
            missing.append('Detector de color')
        if missing:
            diagnostics.append({
                'title': 'Visión parcial',
                'status': 'warn',
                'category': 'Visión',
                'description': 'Faltan componentes de detección visual: ' + ', '.join(missing) + '.',
                'why': 'El sistema no pudo cargar uno o más modelos de visión.',
                'fix': 'Compruebe dependencias, archivos de modelo y configuraciones de detección.',
            })
            vision_status = {'level': 'warn', 'message': 'Visión parcial: ' + ', '.join(missing)}
        else:
            diagnostics.append({
                'title': 'Visión completa',
                'status': 'ok',
                'category': 'Visión',
                'description': 'YOLO y el detector de color están cargados y listos.',
                'why': 'El pipeline de detección se inicializó correctamente.',
                'fix': 'Ninguno.',
            })
            vision_status = {'level': 'ok', 'message': 'Visión operativa'}

    hardware_status = {'level': 'ok', 'message': 'No hay problemas detectados'}
    if getattr(c, 'robot', None) is None:
        diagnostics.append({
            'title': 'Controlador de robot faltante',
            'status': 'error',
            'category': 'Hardware',
            'description': 'El subsistema de control de servos no se inicializó correctamente.',
            'why': 'Puede haber un problema con el controlador PCA9685 o dependencias de I2C.',
            'fix': 'Revise la conexión del PCA9685 y los controladores de hardware.',
        })
        hardware_status = {'level': 'error', 'message': 'Controlador robot no inicializado'}
    else:
        servo_ctrl = getattr(c.robot, 'controlador_servo', None)
        if servo_ctrl is None:
            diagnostics.append({
                'title': 'Controlador de servos inaccesible',
                'status': 'error',
                'category': 'Hardware',
                'description': 'La instancia de ControladorServo no está disponible.',
                'why': 'El subsistema de robot no pudo crear o exponer el controlador de servos.',
                'fix': 'Revise el arranque del sistema y los registros de inicialización.',
            })
            hardware_status = {'level': 'error', 'message': 'Servos no accesibles'}
        else:
            servo_count = len(getattr(servo_ctrl, 'servos', {}) or {})
            diagnostics.append({
                'title': 'Hardware listo',
                'status': 'ok',
                'category': 'Hardware',
                'description': f'Controlador de servos cargado con {servo_count} servos configurados.',
                'why': 'La instancia de ControladorServo está activa.',
                'fix': 'Ninguno.',
            })
            hardware_status = {'level': 'ok', 'message': f'{servo_count} servos configurados'}

    safe = obtener_safe_ctrl()
    if safe.is_emergency:
        diagnostics.append({
            'title': 'Estado SAFE: Emergency Stop',
            'status': 'error',
            'category': 'SafeController',
            'description': 'El modo de emergencia detuvo el robot por seguridad.',
            'why': 'Se ha activado una parada de emergencia manual o automática.',
            'fix': 'Revise la posición física y use Reset Emergencia cuando sea seguro.',
        })
        safe_status = {'level': 'error', 'message': 'Emergency Stop activo'}
    elif safe.is_simulation:
        diagnostics.append({
            'title': 'SafeController en modo simulación',
            'status': 'warn',
            'category': 'SafeController',
            'description': 'Los movimientos se limitan a simulación segura en software.',
            'why': 'El controlador seguro actúa en modo de prueba o sin hardware completo.',
            'fix': 'Desactive el modo de simulación si desea operar el hardware real.',
        })
        safe_status = {'level': 'warn', 'message': 'Modo simulación activo'}
    else:
        diagnostics.append({
            'title': 'SafeController operativo',
            'status': 'ok',
            'category': 'SafeController',
            'description': 'El controlador de seguridad está listo para aceptar comandos.',
            'why': 'No se detectaron condiciones de emergencia.',
            'fix': 'Ninguno.',
        })
        safe_status = {'level': 'ok', 'message': 'SafeController listo'}

    learning_status = {
        'level': 'ok' if _learning_state['enabled'] else 'warn',
        'message': 'Grabación activa' if _learning_state['enabled'] else 'Grabación inactiva',
    }
    if _learning_state['enabled']:
        diagnostics.append({
            'title': 'Modo aprendizaje activo',
            'status': 'warn',
            'category': 'Aprendizaje',
            'description': 'El sistema está grabando eventos de demostración en tiempo real.',
            'why': 'La grabación de demostraciones está habilitada en la interfaz.',
            'fix': 'Use Exportar demo cuando finalice la secuencia.',
        })
    else:
        diagnostics.append({
            'title': 'Aprendizaje en espera',
            'status': 'ok',
            'category': 'Aprendizaje',
            'description': 'El sistema está listo para comenzar a grabar demostraciones.',
            'why': 'No se ha iniciado la grabación de eventos.',
            'fix': 'Actívela desde el panel de Aprendizaje.',
        })

    metrics = _obtener_metricas_sistema()
    if metrics.get('cpu_percent') is not None and metrics['cpu_percent'] > 85:
        diagnostics.append({
            'title': 'Uso de CPU elevado',
            'status': 'warn',
            'category': 'Sistema',
            'description': f'CPU al {metrics["cpu_percent"]:.0f}% indicando presión de carga.',
            'why': 'La aplicación o el sistema ocupan demasiada CPU.',
            'fix': 'Cierre procesos innecesarios o reduzca la carga de visión.',
        })
    if metrics.get('ram_percent') is not None and metrics['ram_percent'] > 90:
        diagnostics.append({
            'title': 'Memoria RAM alta',
            'status': 'warn',
            'category': 'Sistema',
            'description': f'RAM usada al {metrics["ram_percent"]:.0f}%.',
            'why': 'El sistema puede quedarse sin memoria si la tendencia continúa.',
            'fix': 'Libere memoria cerrando aplicaciones o reinicie el sistema.',
        })
    if metrics.get('temperature_celsius') is not None and metrics['temperature_celsius'] >= 72:
        diagnostics.append({
            'title': 'Temperatura elevada',
            'status': 'warn',
            'category': 'Sistema',
            'description': f'Temperatura del sistema: {metrics["temperature_celsius"]:.1f}°C.',
            'why': 'El hardware puede estar caliente debido a carga prolongada.',
            'fix': 'Asegure ventilación adecuada y reduzca la carga si es necesario.',
        })

    counts = {'ok': 0, 'warn': 0, 'error': 0}
    for item in diagnostics:
        counts[item['status']] = counts.get(item['status'], 0) + 1

    return {
        'summary': counts,
        'items': diagnostics,
        'camera_status': camera_status,
        'vision_status': vision_status,
        'hardware_status': hardware_status,
        'safe_status': safe_status,
        'learning_status': learning_status,
        'system_metrics': metrics,
    }


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
    """Genera frames MJPEG continuos desde la cámara con streaming rápido."""
    import cv2
    c = obtener_cerebro()
    frame_count = 0
    error_count = 0
    max_errors = 5
    
    while True:
        try:
            if c.camara is None:
                log.warning("[Video] Cámara no inicializada, reintentando...")
                time.sleep(2)
                continue
                
            img = c._capturar_imagen()
            if img is not None:
                c.frame_actual = img.copy()
                error_count = 0  # Reset error counter on success
                frame_count += 1

                # Dibuja detecciones si están disponibles
                if c.detector_color and (c.objetos or c.recipientes):
                    try:
                        objs_draw = [{'bbox': o.bbox, 'color': o.color,
                                      'clase': o.clase, 'confianza': o.confianza}
                                     for o in c.objetos] if c.objetos else []
                        recs_draw = [{'bbox': r.bbox, 'color': r.color,
                                      'centro': r.centro}
                                     for r in c.recipientes] if c.recipientes else []
                        if objs_draw or recs_draw:
                            img = c.detector_color.dibujar_resultados(img, objs_draw, recs_draw)
                    except Exception as e:
                        log.debug(f"[Video] Error dibujando detecciones: {e}")

                # Añade número de frame al corner inferior derecho
                cv2.putText(img, f"Frame: {frame_count}", (10, img.shape[0] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                _, buffer = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 75])
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
                time.sleep(0.08)  # ~12 FPS para streaming suave
            else:
                error_count += 1
                if error_count > max_errors:
                    log.error(f"[Video] Demasiados errores capturando imagen ({error_count}). Pausa.")
                    time.sleep(3)
                    error_count = 0
                else:
                    time.sleep(0.5)
        except Exception as e:
            log.error(f"[Video] Error en generar_frames: {e}")
            error_count += 1
            time.sleep(1)


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

    estado['learning'] = {
        'enabled': bool(_learning_state['enabled']),
        'recorded_events': len(_learning_state['events']),
    }
    estado['system_metrics'] = _obtener_metricas_sistema()
    estado['diagnostics'] = _diagnosticar_sistema(c)

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
            c.robot.controlador_servo.mover_por_tiempo('gripper', direccion, 1.00, velocidad=0.45)
            _register_learning_event('gripper_manual', {
                'joint': 'gripper',
                'direction': direccion,
                'duration_s': 1.0,
                'speed': 0.45,
            })
            return jsonify({
                'ok': True,
                'msg': f'gripper continuo movido dir={direccion:+d} (1.00s)'
            })
        except Exception as exc:
            return jsonify({
                'ok': False,
                'msg': f'Error moviendo gripper continuo: {exc}'
            })

    delta = direccion * PASO_MANUAL_DEG
    ok = safe.move_relative(joint, delta)

    if ok:
        applied = safe.get_angle(joint)
        _register_learning_event('move_joint', {
            'joint': joint,
            'dir': direccion,
            'delta': delta,
            'angle_applied': applied,
        })
        return jsonify({
            'ok': True,
            'msg': f'{joint} movido {delta:+.0f}° → {applied:.1f}°'
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
    _register_learning_event('set_angle', {
        'joint': joint,
        'target_angle': angle,
        'angle_applied': applied,
    })
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
            _register_learning_event('gripper_continuo', {
                'action': 'stop',
                'speed': 0,
            })
            return jsonify({'ok': True, 'msg': 'gripper continuo detenido'})

        direccion = 1 if speed > 0 else -1
        velocidad = max(0.2, min(1.0, abs(speed) / 100.0))
        c.robot.controlador_servo.mover_por_tiempo('gripper', direccion, 1.00, velocidad=velocidad)
        _register_learning_event('gripper_continuo', {
            'action': 'move',
            'direction': direccion,
            'speed': speed,
            'normalized_speed': velocidad,
            'duration_s': 1.0,
        })
        return jsonify({'ok': True, 'msg': f'gripper continuo speed={speed}'})
    except Exception as exc:
        return jsonify({'ok': False, 'msg': f'Error en gripper continuo: {exc}'})


@app.route('/api/learning/toggle', methods=['POST'])
def api_learning_toggle():
    if _is_calibration_enabled():
        return _calibration_blocked_response('/api/learning/toggle')
    data = request.get_json() or {}
    enable = data.get('enable')
    if enable is None:
        return jsonify({'ok': False, 'msg': 'Falta parámetro enable'})
    enabled = str(enable).lower() in ('1', 'true', 'yes', 'on')
    _learning_state['enabled'] = enabled
    if enabled:
        _learning_state['events'].clear()
        return jsonify({'ok': True, 'msg': 'Grabación de demostración iniciada'})
    return jsonify({
        'ok': True,
        'msg': f'Grabación detenida ({len(_learning_state["events"])} eventos)',
        'count': len(_learning_state['events'])
    })


@app.route('/api/learning/export', methods=['POST'])
def api_learning_export():
    if not _learning_state['events']:
        return jsonify({'ok': False, 'msg': 'No hay eventos grabados para exportar'})
    filename = f'learning_demo_{datetime.utcnow().strftime("%Y%m%d_%H%M%S")}.json'
    filepath = _learning_dir / filename
    with filepath.open('w', encoding='utf-8') as f:
        json.dump({
            'created': datetime.utcnow().isoformat() + 'Z',
            'events': _learning_state['events'],
        }, f, ensure_ascii=False, indent=2)
    return jsonify({'ok': True, 'msg': 'Demostración guardada', 'filename': filename, 'count': len(_learning_state['events'])})


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
<title>Brazo Robótico Autónomo - Control IA</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Inter','Segoe UI',system-ui,sans-serif;background:linear-gradient(135deg,#0f172a 0%,#1a1f3a 100%);color:#e2e8f0;min-height:100vh}
html{scroll-behavior:smooth}

@keyframes fadeIn{from{opacity:0;transform:translateY(10px)}to{opacity:1;transform:translateY(0)}}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.7}}
@keyframes glow{0%,100%{box-shadow:0 0 20px rgba(56,189,248,.1)}50%{box-shadow:0 0 30px rgba(56,189,248,.3)}}
@keyframes slideDown{from{opacity:0;transform:translateY(-10px)}to{opacity:1;transform:translateY(0)}}

.anim-fadeIn{animation:fadeIn .4s ease-out}
.anim-glow{animation:glow 2s infinite}
.anim-pulse{animation:pulse 2s infinite}
.top-bar{background:linear-gradient(135deg,#1e293b 0%,#0f172a 100%);padding:20px 32px;border-bottom:2px solid rgba(56,189,248,.2);box-shadow:0 8px 32px rgba(0,0,0,.3)}
.top-bar h1{font-size:1.8rem;font-weight:800;background:linear-gradient(90deg,#38bdf8,#a78bfa);-webkit-background-clip:text;-webkit-text-fill-color:transparent;margin-bottom:8px}
.top-bar-row{display:flex;align-items:center;justify-content:space-between;gap:20px}
.estado-badge{padding:6px 16px;border-radius:20px;font-size:.75rem;font-weight:700;text-transform:uppercase;letter-spacing:.6px;background:rgba(56,189,248,.15);color:#38bdf8;border:1px solid rgba(56,189,248,.3)}
.top-bar-diag{margin-top:12px;padding-top:12px;border-top:1px solid rgba(56,189,248,.1);display:flex;gap:16px;font-size:.75rem;color:#cbd5e1}
.diag-pill{padding:4px 12px;border-radius:12px;font-weight:700;font-size:.7rem}
.diag-pill.ok{background:#14532d;color:#bbf7d0}
.diag-pill.warn{background:#f59e0b;color:#1f2937}
.diag-pill.bad{background:#7f1d1d;color:#fecaca}
.cal-banner{display:none;margin-top:12px;padding:12px 16px;border:1px solid #ef4444;background:rgba(239,68,68,.1);color:#fca5a5;border-radius:8px;font-size:.8rem;font-weight:700}
.main{display:grid;grid-template-columns:1fr 420px;gap:20px;padding:20px 32px;max-width:1600px;margin:0 auto}
@media(max-width:1200px){.main{grid-template-columns:1fr;gap:16px}}
.video-section{background:rgba(30,41,59,.6);border-radius:16px;overflow:hidden;border:1px solid rgba(56,189,248,.15);box-shadow:0 8px 32px rgba(56,189,248,.05);animation:fadeIn .6s ease-out}
.video-panel img{width:100%;display:block;min-height:400px;background:#000;object-fit:cover}
.video-panel img.invert{transform:rotate(180deg);transition:transform .2s ease}
.controls-grid{display:grid;gap:16px}
.side{display:flex;flex-direction:column;gap:16px}
.card{background:rgba(30,41,59,.6);border-radius:16px;padding:20px;border:1px solid rgba(56,189,248,.1);backdrop-filter:blur(10px);transition:all .3s}
.card:hover{border-color:rgba(56,189,248,.3);box-shadow:0 8px 32px rgba(56,189,248,.1)}
.card h3{font-size:.8rem;text-transform:uppercase;letter-spacing:.8px;color:#a78bfa;margin-bottom:12px;font-weight:700}
.btn-grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.btn{padding:12px;border:none;border-radius:10px;font-weight:700;font-size:.85rem;cursor:pointer;transition:all .3s;position:relative;overflow:hidden}
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

<div class="top-bar anim-fadeIn">
  <div class="top-bar-row">
    <div>
      <h1>⚙️ BRAZO ROBÓTICO IA</h1>
      <p style="font-size:.8rem;color:#94a3b8;margin:4px 0 0">Control en tiempo real con detección visual</p>
    </div>
    <span class="estado-badge" id="badge-estado">IDLE</span>
  </div>
  <div class="top-bar-diag">
    <span>Cámara: <span class="diag-pill ok" id="diag-camera-pill">—</span></span>
    <span>Visión: <span class="diag-pill ok" id="diag-vision-pill">—</span></span>
    <span>Hardware: <span class="diag-pill ok" id="diag-hardware-pill">—</span></span>
    <span id="diag-escaneo-motivo" title="Estado último escaneo" style="font-size:.7rem;color:#cbd5e1">—</span>
  </div>
  <div class="cal-banner" id="cal-banner">🔒 MODO CALIBRACIÓN ACTIVO — Movimiento bloqueado</div>
</div>

<div class="main">
  <div class="controls-grid">
    <div class="video-section">
      <div class="video-panel">
        <img id="video" src="/video_feed" alt="Stream de cámara en vivo">
      </div>
      <div style="padding:10px;display:flex;gap:8px;align-items:center">
        <button class="btn" id="btn-invert" style="padding:8px;font-size:.85rem" onclick="toggleInvert()">🔄 Invertir imagen</button>
        <small style="color:#94a3b8">Girar 180° (útil si la cámara está invertida)</small>
      </div>
    </div>

    <script>
    // Toggle display-only inversion for the stream image (persists in localStorage)
    function toggleInvert(){
      const img = document.getElementById('video');
      if(!img) return;
      img.classList.toggle('invert');
      const inv = img.classList.contains('invert');
      try{ localStorage.setItem('video_invert', inv? '1':'0'); }catch(e){}
      const btn = document.getElementById('btn-invert');
      if(btn) btn.textContent = inv? '🔄 Invertida' : '🔄 Invertir imagen';
    }
    (function(){
      try{
        const v = localStorage.getItem('video_invert');
        if(v==='1'){
          const img = document.getElementById('video');
          if(img) img.classList.add('invert');
          const btn = document.getElementById('btn-invert');
          if(btn) btn.textContent = '🔄 Invertida';
        }
      }catch(e){}
    })();
    </script>

    <div class="card anim-fadeIn">
      <h3>📊 Estado del Brazo</h3>
      <svg viewBox="0 0 300 320" style="width:100%;height:auto;min-height:200px">
        <circle cx="150" cy="280" r="15" fill="#38bdf8" opacity=".3"/>
        <circle cx="150" cy="280" r="12" fill="#38bdf8"/>
        <text x="150" y="305" text-anchor="middle" font-size="12" fill="#cbd5e1">Base</text>
        <line x1="150" y1="280" x2="150" y2="210" stroke="#818cf8" stroke-width="8" stroke-linecap="round"/>
        <circle cx="150" cy="210" r="10" fill="#818cf8"/>
        <text x="120" y="215" font-size="11" fill="#a78bfa">Hombro</text>
        <line x1="150" y1="210" x2="180" y2="130" stroke="#a78bfa" stroke-width="8" stroke-linecap="round"/>
        <circle cx="180" cy="130" r="10" fill="#a78bfa"/>
        <text x="195" y="135" font-size="11" fill="#cbd5e1">Codo</text>
        <line x1="180" y1="130" x2="200" y2="60" stroke="#c4b5fd" stroke-width="6" stroke-linecap="round"/>
        <circle cx="200" cy="60" r="8" fill="#c4b5fd"/>
        <line x1="200" y1="60" x2="210" y2="40" stroke="#e9d5ff" stroke-width="4" stroke-linecap="round"/>
        <circle cx="210" cy="40" r="6" fill="#e9d5ff"/>
        <text x="215" y="45" font-size="10" fill="#cbd5e1">Pinza</text>
      </svg>
      <div id="pos-bars" style="margin-top:12px;display:grid;gap:8px">
        <div style="display:grid;grid-template-columns:80px 1fr 50px;align-items:center;gap:8px">
          <span style="font-size:.75rem;font-weight:600;color:#818cf8">Hombro</span>
          <div style="height:6px;background:rgba(56,189,248,.2);border-radius:3px;overflow:hidden">
            <div id="pos-shoulder" style="height:100%;background:linear-gradient(90deg,#38bdf8,#818cf8);width:50%;transition:width .3s"></div>
          </div>
          <span id="pv-shoulder" style="font-size:.7rem;font-weight:700;color:#38bdf8;text-align:right">90°</span>
        </div>
        <div style="display:grid;grid-template-columns:80px 1fr 50px;align-items:center;gap:8px">
          <span style="font-size:.75rem;font-weight:600;color:#a78bfa">Codo</span>
          <div style="height:6px;background:rgba(167,139,250,.2);border-radius:3px;overflow:hidden">
            <div id="pos-elbow" style="height:100%;background:linear-gradient(90deg,#a78bfa,#c4b5fd);width:50%;transition:width .3s"></div>
          </div>
          <span id="pv-elbow" style="font-size:.7rem;font-weight:700;color:#a78bfa;text-align:right">90°</span>
        </div>
        <div style="display:grid;grid-template-columns:80px 1fr 50px;align-items:center;gap:8px">
          <span style="font-size:.75rem;font-weight:600;color:#c4b5fd">Muñeca</span>
          <div style="height:6px;background:rgba(196,181,253,.2);border-radius:3px;overflow:hidden">
            <div id="pos-wrist" style="height:100%;background:linear-gradient(90deg,#c4b5fd,#e9d5ff);width:50%;transition:width .3s"></div>
          </div>
          <span id="pv-wrist" style="font-size:.7rem;font-weight:700;color:#c4b5fd;text-align:right">90°</span>
        </div>
      </div>
    </div>

    <div class="card anim-fadeIn">
      <h3>🎮 Control Principal</h3>
      <div class="btn-grid">
        <button class="btn" style="background:linear-gradient(135deg,#16a34a,#22c55e);color:#fff;border:1px solid #22c55e" onclick="apiPost('/api/iniciar')">▶️ Iniciar</button>
        <button class="btn" style="background:linear-gradient(135deg,#ea580c,#f97316);color:#fff;border:1px solid #f97316" onclick="apiPost('/api/pausar')">⏸ Pausar</button>
        <button class="btn" style="background:linear-gradient(135deg,#0891b2,#06b6d4);color:#fff;border:1px solid #06b6d4" onclick="apiPost('/api/reanudar')">▶️ Reanudar</button>
        <button class="btn" style="background:linear-gradient(135deg,#b91c1c,#dc2626);color:#fff;border:1px solid #dc2626" onclick="apiPost('/api/detener')">⏹ Detener</button>
        <button class="btn" style="background:linear-gradient(135deg,#7c3aed,#a855f7);color:#fff;border:1px solid #a855f7;grid-column:1/-1" onclick="apiPost('/api/home')">🏠 HOME</button>
      </div>
    </div>

    <div class="card anim-fadeIn">
      <h3>🔧 Control Manual (°)</h3>
      <div style="display:grid;gap:10px">
        <div style="display:grid;grid-template-columns:90px 1fr 50px;align-items:center;gap:8px">
          <label style="font-size:.75rem;font-weight:600;color:#cbd5e1">Base</label>
          <input id="sl-base" type="range" min="0" max="180" step="1" value="90" oninput="sliderMoveJoint('base', this.value)" style="cursor:pointer">
          <span class="slider-val" id="slv-base" style="font-size:.7rem;font-weight:700;color:#38bdf8;text-align:right">90°</span>
        </div>
        <div style="display:grid;grid-template-columns:90px 1fr 50px;align-items:center;gap:8px">
          <label style="font-size:.75rem;font-weight:600;color:#cbd5e1">Hombro</label>
          <input id="sl-shoulder" type="range" min="15" max="165" step="1" value="90" oninput="sliderMoveJoint('shoulder', this.value)" style="cursor:pointer">
          <span class="slider-val" id="slv-shoulder" style="font-size:.7rem;font-weight:700;color:#38bdf8;text-align:right">90°</span>
        </div>
        <div style="display:grid;grid-template-columns:90px 1fr 50px;align-items:center;gap:8px">
          <label style="font-size:.75rem;font-weight:600;color:#cbd5e1">Codo</label>
          <input id="sl-elbow" type="range" min="20" max="160" step="1" value="90" oninput="sliderMoveJoint('elbow', this.value)" style="cursor:pointer">
          <span class="slider-val" id="slv-elbow" style="font-size:.7rem;font-weight:700;color:#38bdf8;text-align:right">90°</span>
        </div>
        <div style="display:grid;grid-template-columns:90px 1fr 50px;align-items:center;gap:8px">
          <label style="font-size:.75rem;font-weight:600;color:#cbd5e1">Muñeca</label>
          <input id="sl-wrist" type="range" min="20" max="170" step="1" value="90" oninput="sliderMoveJoint('wrist', this.value)" style="cursor:pointer">
          <span class="slider-val" id="slv-wrist" style="font-size:.7rem;font-weight:700;color:#38bdf8;text-align:right">90°</span>
        </div>
        <div style="display:grid;grid-template-columns:90px 1fr 50px;align-items:center;gap:8px">
          <label style="font-size:.75rem;font-weight:600;color:#cbd5e1">Pinza</label>
          <input id="sl-gripper" type="range" min="-100" max="100" step="1" value="0" oninput="sliderMoveGripper(this.value)" onchange="sliderStopGripper()" style="cursor:pointer">
          <span class="slider-val" id="slv-gripper" style="font-size:.7rem;font-weight:700;color:#38bdf8;text-align:right">0%</span>
        </div>
      </div>
    </div>
  </div>

  <div class="side anim-fadeIn">
    <div class="card">
      <h3>⚡ Estadísticas</h3>
      <div style="display:grid;grid-template-columns:repeat(2,1fr);gap:12px">
        <div style="background:rgba(56,189,248,.1);padding:12px;border-radius:8px;text-align:center">
          <div style="font-size:1.8rem;font-weight:800;color:#38bdf8" id="st-detectados">0</div>
          <div style="font-size:.7rem;color:#94a3b8;margin-top:4px">Detectados</div>
        </div>
        <div style="background:rgba(34,197,94,.1);padding:12px;border-radius:8px;text-align:center">
          <div style="font-size:1.8rem;font-weight:800;color:#22c55e" id="st-exitos">0</div>
          <div style="font-size:.7rem;color:#94a3b8;margin-top:4px">Agarres OK</div>
        </div>
        <div style="background:rgba(239,68,68,.1);padding:12px;border-radius:8px;text-align:center">
          <div style="font-size:1.8rem;font-weight:800;color:#ef4444" id="st-fallos">0</div>
          <div style="font-size:.7rem;color:#94a3b8;margin-top:4px">Fallos</div>
        </div>
        <div style="background:rgba(168,85,247,.1);padding:12px;border-radius:8px;text-align:center">
          <div style="font-size:1.8rem;font-weight:800;color:#a855f7" id="st-depositos">0</div>
          <div style="font-size:.7rem;color:#94a3b8;margin-top:4px">Depósitos</div>
        </div>
      </div>
    </div>

    <div class="card">
      <h3>🎯 Objetos Detectados</h3>
      <div id="obj-list" style="max-height:120px;overflow-y:auto;display:grid;gap:6px">
        <em style="color:#64748b;font-size:.8rem">Sin datos</em>
      </div>
    </div>

    <div class="card">
      <h3>📦 Recipientes</h3>
      <div id="rec-list" style="max-height:120px;overflow-y:auto;display:grid;gap:6px">
        <em style="color:#64748b;font-size:.8rem">Sin datos</em>
      </div>
    </div>

    <div class="card">
      <h3>🔴 Estado Sistema</h3>
      <div id="safe-status" style="font-size:.8rem;color:#22c55e;font-weight:600;min-height:20px">✓ SAFE: OK</div>
      <button class="btn" id="btn-reset-emerg" style="display:none;margin-top:8px;width:100%;background:linear-gradient(135deg,#7c3aed,#a855f7);color:#fff;border:1px solid #a855f7" onclick="resetEmergencia()">Reset Emergencia</button>
    </div>

    <div class="card">
      <h3>Centro de Diagnóstico</h3>
      <div class="stat-grid">
        <div class="stat"><div class="val" id="health-errors">0</div><div class="lbl">Errores</div></div>
        <div class="stat"><div class="val" id="health-warnings">0</div><div class="lbl">Advertencias</div></div>
        <div class="stat"><div class="val" id="health-ok">0</div><div class="lbl">Sistemas OK</div></div>
      </div>
      <div style="margin-top:12px;line-height:1.5;font-size:.82rem;color:#cbd5e1">
        <div><strong>Cámara:</strong> <span id="diag-camera-status">—</span></div>
        <div><strong>Visión:</strong> <span id="diag-vision-status">—</span></div>
        <div><strong>Hardware:</strong> <span id="diag-hardware-status">—</span></div>
      </div>
      <div id="diag-list" style="margin-top:12px;font-size:.78rem;color:#cbd5e1;line-height:1.5;max-height:160px;overflow-y:auto"></div>
    </div>

    <div class="card">
      <h3>Aprendizaje por demostración</h3>
      <div style="display:flex;gap:8px;flex-wrap:wrap;margin-bottom:10px">
        <button class="btn-sm" id="btn-learning-toggle" style="background:#2563eb" onclick="toggleLearning()">Grabar demo</button>
        <button class="btn-sm" style="background:#047857" onclick="exportLearning()">Exportar demo</button>
      </div>
      <div style="font-size:.78rem;color:#cbd5e1;line-height:1.45">
        Estado: <strong id="learning-status">OFF</strong>
        <div id="learning-count" style="margin-top:4px">0 eventos</div>
      </div>
    </div>

    <div class="card">
      <h3>⚠️ Emergencia</h3>
      <button class="btn" style="width:100%;background:linear-gradient(135deg,#dc2626,#ef4444);color:#fff;border:1px solid #ef4444;font-weight:800" onclick="apiPost('/api/emergencia')">🛑 PARADA TOTAL</button>
    </div>
  </div>
</div>

<script>
let CAL_MODE_ACTIVE = false;
function apiPost(url, body){
  fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:body?JSON.stringify(body):'{}'})
  .then(r=>r.json()).then(d=>{if(d.msg)console.log(d.msg)}).catch(e=>console.error(e));
}
function sliderMoveJoint(joint, value){
  if(CAL_MODE_ACTIVE) return;
  const v=Math.round(Number(value)||0);
  const lbl=document.getElementById('slv-'+joint);
  if(lbl) lbl.textContent=v+'°';
  setTimeout(()=>apiPost('/api/set_angle',{joint,angle:v}), 50);
}
function sliderMoveGripper(value){
  if(CAL_MODE_ACTIVE) return;
  const v=Math.round(Number(value)||0);
  const lbl=document.getElementById('slv-gripper');
  if(lbl) lbl.textContent=v+'%';
  setTimeout(()=>apiPost('/api/gripper_continuo',{speed:v}), 50);
}
function sliderStopGripper(){
  if(CAL_MODE_ACTIVE) return;
  const sl=document.getElementById('sl-gripper');
  const lbl=document.getElementById('slv-gripper');
  if(sl) sl.value = 0;
  if(lbl) lbl.textContent='0%';
  apiPost('/api/gripper_continuo',{speed:0});
}
function resetEmergencia(){
  if(!confirm('¿Confirmas que el brazo está seguro? Se reactivarán los movimientos.'))return;
  apiPost('/api/reset_emergency');
}
function actualizarUI(){
  fetch('/api/estado').then(r=>r.json()).then(d=>{
    const badge=document.getElementById('badge-estado');
    badge.textContent=d.estado;
    const cameraPill=document.getElementById('diag-camera-pill');
    const visionPill=document.getElementById('diag-vision-pill');
    const hardwarePill=document.getElementById('diag-hardware-pill');
    const mot=document.getElementById('diag-escaneo-motivo');
    if(d.diagnostics){
      if(cameraPill && d.diagnostics.camera_status){
        cameraPill.textContent=d.diagnostics.camera_status.message;
        cameraPill.className='diag-pill '+d.diagnostics.camera_status.level;
      }
      if(visionPill && d.diagnostics.vision_status){
        visionPill.textContent=d.diagnostics.vision_status.message;
        visionPill.className='diag-pill '+d.diagnostics.vision_status.level;
      }
      if(hardwarePill && d.diagnostics.hardware_status){
        hardwarePill.textContent=d.diagnostics.hardware_status.message;
        hardwarePill.className='diag-pill '+d.diagnostics.hardware_status.level;
      }
    } else if(cameraPill && typeof d.ultimo_escaneo_ok==='boolean'){
      cameraPill.textContent=d.ultimo_escaneo_ok?'OK':'Falla';
      cameraPill.className='diag-pill '+(d.ultimo_escaneo_ok?'ok':'bad');
    }
    if(mot && d.ultimo_escaneo_motivo!==undefined){
      mot.textContent=d.ultimo_escaneo_motivo;
    }
    const s=d.estadisticas;
    document.getElementById('st-detectados').textContent=s.objetos_detectados;
    document.getElementById('st-exitos').textContent=s.agarres_exitosos;
    document.getElementById('st-fallos').textContent=s.agarres_fallidos;
    document.getElementById('st-depositos').textContent=s.depositos_exitosos;
    const learningStatus=document.getElementById('learning-status');
    const learningCount=document.getElementById('learning-count');
    if(learningStatus) learningStatus.textContent = d.learning && d.learning.enabled ? 'ON' : 'OFF';
    if(learningCount) learningCount.textContent = d.learning ? d.learning.recorded_events + ' eventos' : '0 eventos';
    if(d.diagnostics){
      const health = d.diagnostics.summary || {ok:0,warn:0,error:0};
      const errors = document.getElementById('health-errors');
      const warns = document.getElementById('health-warnings');
      const oks = document.getElementById('health-ok');
      if(errors) errors.textContent = health.error;
      if(warns) warns.textContent = health.warn;
      if(oks) oks.textContent = health.ok;
      const cameraStatus = document.getElementById('diag-camera-status');
      const visionStatus = document.getElementById('diag-vision-status');
      const hardwareStatus = document.getElementById('diag-hardware-status');
      if(cameraStatus && d.diagnostics.camera_status) cameraStatus.textContent = d.diagnostics.camera_status.message;
      if(visionStatus && d.diagnostics.vision_status) visionStatus.textContent = d.diagnostics.vision_status.message;
      if(hardwareStatus && d.diagnostics.hardware_status) hardwareStatus.textContent = d.diagnostics.hardware_status.message;
      const diagList = document.getElementById('diag-list');
      if(diagList){
        diagList.innerHTML = d.diagnostics.items.slice(0,6).map(item => {
          const color = item.status === 'error' ? '#fca5a5' : item.status === 'warn' ? '#fde68a' : '#86efac';
          return `<div style="margin-bottom:10px"><span style="font-weight:700;color:${color}">${item.title}</span><div style="font-size:.78rem;color:#cbd5e1;margin-top:4px">${item.description}</div></div>`;
        }).join('');
      }
    }
    if(d.posiciones){
      ['shoulder','elbow','wrist'].forEach(j=>{
        const v=d.posiciones[j];
        if(v!==undefined){
          const pct=Math.round(v*100);
          const deg=Math.round(v*180);
          const bar=document.getElementById('pos-'+j);
          const lbl=document.getElementById('pv-'+j);
          if(bar)bar.style.width=pct+'%';
          if(lbl)lbl.textContent=deg+'°';
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
    const ol=document.getElementById('obj-list');
    if(d.objetos.length){
      ol.innerHTML=d.objetos.map(o=>`<div style="font-size:.75rem;color:#cbd5e1;padding:6px;background:rgba(56,189,248,.1);border-radius:6px">${o.clase} <span style="color:#38bdf8;font-weight:700">${(o.confianza*100).toFixed(0)}%</span></div>`).join('');
    } else {ol.innerHTML='<em style="color:#64748b;font-size:.8rem">Ninguno</em>'}
    const rl=document.getElementById('rec-list');
    if(d.recipientes.length){
      rl.innerHTML=d.recipientes.map(r=>`<div style="font-size:.75rem;color:#cbd5e1;padding:6px;background:rgba(167,139,250,.1);border-radius:6px">${r.color} <span style="color:#a78bfa;font-weight:700">${r.depositados} obj</span></div>`).join('');
    } else {rl.innerHTML='<em style="color:#64748b;font-size:.8rem">Ninguno</em>'}
    const safeStatus=document.getElementById('safe-status');
    const btnReset=document.getElementById('btn-reset-emerg');
    if(safeStatus){
      if(d.safe_emergency){
        safeStatus.innerHTML='<span style="color:#ef4444;font-weight:700">⚡ Emergency Stop ACTIVO</span>';
        if(btnReset)btnReset.style.display='block';
      } else {
        safeStatus.innerHTML='<span style="color:#22c55e">✓ SAFE: OK</span>';
        if(btnReset)btnReset.style.display='none';
      }
    }
    CAL_MODE_ACTIVE = !!d.calibration_mode;
    const banner=document.getElementById('cal-banner');
    if(banner) banner.style.display = CAL_MODE_ACTIVE ? 'block' : 'none';
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
