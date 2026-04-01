#!/usr/bin/env python3
"""
CEREBRO AUTONOMO DEL BRAZO ROBOTICO
====================================
Ciclo principal:
  1. Escanear area de trabajo con la camara
  2. Detectar objetos (YOLO) y determinar su color (HSV)
  3. Localizar recipientes por color
  4. Planificar ruta de pick & place
  5. Ejecutar con reintentos y recuperacion de errores
  6. Repetir hasta que no queden objetos

Filosofia: si algo falla, se reintenta con una estrategia diferente.
Nunca se queda bloqueado: escala de micro-ajuste -> reintento -> reposicion -> skip.
"""

import time
import os
import sys
import json
import logging as log
import threading
from datetime import datetime
from enum import Enum, auto

log.basicConfig(level=log.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from control.robot_controller import ControladorRobotico
from config_sistema import (
    STEPPER_HABILITADO,
    CAMARA_HABILITADA,
    CONFIANZA_MINIMA_DETECCION,
    MAX_REINTENTOS_AGARRE,
    VELOCIDAD_AUTONOMA,
    PERMITIR_DETECCION_SIMULADA,
)


class EstadoRobot(Enum):
    IDLE = auto()
    ESCANEANDO = auto()
    VISION_FALLA = auto()
    PLANIFICANDO = auto()
    RECOGIENDO = auto()
    TRANSPORTANDO = auto()
    DEPOSITANDO = auto()
    RECUPERANDO_ERROR = auto()
    PAUSADO = auto()
    COMPLETADO = auto()


class ObjetoDetectado:
    """Representa un objeto encontrado en el area de trabajo."""
    __slots__ = ('clase', 'color', 'confianza', 'bbox', 'centro',
                 'posicion_relativa', 'intentos_agarre', 'agarrado')

    def __init__(self, clase, color, confianza, bbox, centro, posicion_relativa):
        self.clase = clase
        self.color = color
        self.confianza = confianza
        self.bbox = bbox
        self.centro = centro
        self.posicion_relativa = posicion_relativa
        self.intentos_agarre = 0
        self.agarrado = False

    def to_dict(self):
        return {
            'clase': self.clase, 'color': self.color,
            'confianza': round(self.confianza, 3),
            'centro': self.centro,
            'posicion_relativa': round(self.posicion_relativa, 3),
            'intentos': self.intentos_agarre, 'agarrado': self.agarrado,
        }


class Recipiente:
    """Representa un recipiente de color donde depositar objetos."""
    __slots__ = ('color', 'bbox', 'centro', 'posicion_relativa', 'objetos_depositados')

    def __init__(self, color, bbox, centro, posicion_relativa):
        self.color = color
        self.bbox = bbox
        self.centro = centro
        self.posicion_relativa = posicion_relativa
        self.objetos_depositados = 0

    def to_dict(self):
        return {
            'color': self.color, 'centro': self.centro,
            'posicion_relativa': round(self.posicion_relativa, 3),
            'depositados': self.objetos_depositados,
        }


class CerebroAutonomo:
    """Orquestador principal del brazo robotico autonomo."""

    FACTOR_PASOS_BASE = 400

    def __init__(self, habilitar_hardware=True):
        self.estado = EstadoRobot.IDLE
        self.robot = None
        self.camara = None
        self.detector_yolo = None
        self.detector_color = None

        self.objetos = []
        self.recipientes = []
        self.historial = []
        self.frame_actual = None

        self._pausar = False
        self._detener = False
        self._lock = threading.Lock()
        self._ultimo_escaneo_ok = True
        self._ultimo_escaneo_motivo = 'ok'

        self.estadisticas = {
            'objetos_detectados': 0,
            'agarres_exitosos': 0,
            'agarres_fallidos': 0,
            'depositos_exitosos': 0,
            'errores_recuperados': 0,
            'ciclos_completados': 0,
        }

        if habilitar_hardware:
            self._inicializar_hardware()

    def _inicializar_hardware(self):
        """Inicializa controladores, camara y modelos de IA."""
        log.info("Inicializando hardware...")

        try:
            self.robot = ControladorRobotico(habilitar_stepper=STEPPER_HABILITADO)
            log.info("Controlador robotico OK")
        except Exception as e:
            log.error(f"Error inicializando controlador: {e}")
            raise

        if CAMARA_HABILITADA:
            try:
                from perception.vision.camera.main import CameraManager
                self.camara = CameraManager()
                log.info("Camara OK")
            except Exception as e:
                log.warning(f"Camara no disponible: {e} -- modo sin vision")

        try:
            from perception.vision.detection.main import DetectionModel
            self.detector_yolo = DetectionModel()
            log.info("YOLO OK")
        except Exception as e:
            log.warning(f"YOLO no disponible: {e}")

        try:
            from perception.vision.color_detector import DetectorColor
            self.detector_color = DetectorColor()
            log.info("Detector de color OK")
        except Exception as e:
            log.warning(f"Detector de color no disponible: {e}")

    # ------------------------------------------------------------------
    # CICLO PRINCIPAL
    # ------------------------------------------------------------------

    def ejecutar_ciclo_autonomo(self, max_ciclos=50):
        """Bucle principal: escanea, planifica, ejecuta pick&place, repite."""
        log.info("=== INICIO MODO AUTONOMO ===")
        self._detener = False
        ciclo = 0

        try:
            self.robot.posicion_home()
            time.sleep(1)

            while ciclo < max_ciclos and not self._detener:
                ciclo += 1
                log.info(f"\n{'='*50}")
                log.info(f"CICLO {ciclo}/{max_ciclos}")
                log.info(f"{'='*50}")

                self._esperar_si_pausado()
                if self._detener:
                    break

                self._cambiar_estado(EstadoRobot.ESCANEANDO)
                objetos, recipientes = self._escanear_entorno()
                if not self._ultimo_escaneo_ok:
                    self._cambiar_estado(EstadoRobot.VISION_FALLA)
                    log.warning(
                        "Fallo de vision (%s). Reintentando escaneo en 2s...",
                        self._ultimo_escaneo_motivo,
                    )
                    time.sleep(2)
                    objetos, recipientes = self._escanear_entorno()
                    if not self._ultimo_escaneo_ok:
                        self.estadisticas['errores_recuperados'] += 1
                        log.warning(
                            "Vision no recuperada (%s). Saltando ciclo para evitar decisiones a ciegas.",
                            self._ultimo_escaneo_motivo,
                        )
                        time.sleep(2)
                        continue

                if not objetos:
                    log.info("No se detectaron objetos. Reescaneando en 3s...")
                    time.sleep(3)
                    objetos, recipientes = self._escanear_entorno()
                    if not self._ultimo_escaneo_ok:
                        self._cambiar_estado(EstadoRobot.VISION_FALLA)
                        self.estadisticas['errores_recuperados'] += 1
                        log.warning(
                            "Segundo escaneo con fallo de vision (%s). No se marca area limpia.",
                            self._ultimo_escaneo_motivo,
                        )
                        time.sleep(2)
                        continue
                    if not objetos:
                        log.info("Area limpia: no hay objetos que recoger.")
                        self._cambiar_estado(EstadoRobot.COMPLETADO)
                        break

                self._cambiar_estado(EstadoRobot.PLANIFICANDO)
                plan = self._planificar(objetos, recipientes)

                if not plan:
                    log.warning("No se pudo generar plan. Saltando ciclo.")
                    time.sleep(2)
                    continue

                for tarea in plan:
                    if self._detener:
                        break
                    self._esperar_si_pausado()
                    self._ejecutar_tarea(tarea)

                self.estadisticas['ciclos_completados'] += 1
                self.robot.posicion_home()
                time.sleep(1)

        except KeyboardInterrupt:
            log.info("Interrumpido por usuario")
        except Exception as e:
            log.error(f"Error critico en ciclo autonomo: {e}")
            self._cambiar_estado(EstadoRobot.RECUPERANDO_ERROR)
            self.robot._posicion_segura()
        finally:
            self._cambiar_estado(EstadoRobot.IDLE)
            log.info(f"\n{'='*50}")
            log.info("ESTADISTICAS FINALES:")
            for k, v in self.estadisticas.items():
                log.info(f"  {k}: {v}")
            log.info(f"{'='*50}")

    # ------------------------------------------------------------------
    # ESCANEO Y DETECCION
    # ------------------------------------------------------------------

    def _escanear_entorno(self):
        """Captura imagen, detecta objetos con YOLO, clasifica colores,
        busca recipientes. Retorna (objetos, recipientes)."""
        self.objetos = []
        self.recipientes = []
        self._ultimo_escaneo_ok = True
        self._ultimo_escaneo_motivo = 'ok'

        self.robot.posicion_escaneo()
        time.sleep(0.8)

        imagen = self._capturar_imagen()
        if imagen is None:
            log.warning("No se pudo capturar imagen")
            self._ultimo_escaneo_ok = False
            self._ultimo_escaneo_motivo = 'captura_imagen'
            return [], []

        self.frame_actual = imagen.copy()
        objetos_detectados = self._detectar_objetos(imagen)
        recipientes_detectados = self._detectar_recipientes(imagen)

        self.objetos = objetos_detectados
        self.recipientes = recipientes_detectados
        self.estadisticas['objetos_detectados'] += len(objetos_detectados)

        log.info(f"Escaneo: {len(objetos_detectados)} objetos, {len(recipientes_detectados)} recipientes")
        for obj in objetos_detectados:
            log.info(f"  Objeto: {obj.clase} color={obj.color} conf={obj.confianza:.2f} pos={obj.posicion_relativa:.2f}")
        for rec in recipientes_detectados:
            log.info(f"  Recipiente: color={rec.color} pos={rec.posicion_relativa:.2f}")

        return objetos_detectados, recipientes_detectados

    def _fallback_objetos(self, razon: str):
        """Si PERMITIR_DETECCION_SIMULADA: datos ficticios para pruebas. Si no: lista vacía (seguro en hardware)."""
        if PERMITIR_DETECCION_SIMULADA:
            log.info("Deteccion simulada de objetos (%s)", razon)
            return self._deteccion_simulada()
        self._ultimo_escaneo_ok = False
        self._ultimo_escaneo_motivo = f'vision_objetos:{razon}'
        log.warning(
            "PERMITIR_DETECCION_SIMULADA=False: sin objetos reales (%s). No se inventan posiciones.",
            razon,
        )
        return []

    def _fallback_recipientes(self, razon: str):
        if PERMITIR_DETECCION_SIMULADA:
            return self._recipientes_por_defecto()
        self._ultimo_escaneo_ok = False
        self._ultimo_escaneo_motivo = f'vision_recipientes:{razon}'
        log.warning(
            "PERMITIR_DETECCION_SIMULADA=False: sin recipientes detectados (%s).",
            razon,
        )
        return []

    def _capturar_imagen(self):
        """Captura imagen desde la camara."""
        if self.camara is None:
            return None
        try:
            resultado = self.camara.capture_image(save=False)
            if isinstance(resultado, tuple):
                imagen = resultado[0]
            else:
                imagen = resultado
            return imagen
        except Exception as e:
            log.error(f"Error capturando imagen: {e}")
            return None

    def _detectar_objetos(self, imagen):
        """Usa YOLO + color para detectar y clasificar objetos."""
        if self.detector_yolo is None or self.detector_color is None:
            return self._fallback_objetos('YOLO o detector de color no disponible')

        objetos = []
        try:
            ancho = imagen.shape[1]

            modelo = self.detector_yolo.object_model
            resultados = modelo.predict(
                imagen, conf=CONFIANZA_MINIMA_DETECCION,
                verbose=False, imgsz=640, stream=False, task='detect'
            )
            nombres = modelo.names

            if resultados is None:
                return self._fallback_objetos('resultados YOLO nulos')

            for res in resultados:
                if res.boxes is None or len(res.boxes) == 0:
                    continue
                for box in res.boxes:
                    conf = float(box.conf[0])
                    cls_id = int(box.cls[0])
                    clase = nombres.get(cls_id, f'clase_{cls_id}') if isinstance(nombres, dict) else str(cls_id)
                    xyxy = box.xyxy[0].cpu().numpy()
                    x1, y1, x2, y2 = xyxy

                    color, pct = self.detector_color.color_dominante_region(
                        imagen, xyxy, clase_yolo=clase)
                    centro = (int((x1 + x2) / 2), int((y1 + y2) / 2))
                    pos_rel = self.detector_color.posicion_relativa_en_imagen(centro, ancho)

                    objetos.append(ObjetoDetectado(
                        clase=clase, color=color, confianza=conf,
                        bbox=(x1, y1, x2, y2), centro=centro,
                        posicion_relativa=pos_rel,
                    ))

            log.info(f"YOLO detecto {len(objetos)} objetos reales")

        except Exception as e:
            log.error(f"Error en deteccion YOLO: {e}")
            import traceback
            traceback.print_exc()
            return self._fallback_objetos('excepcion en deteccion YOLO')

        return objetos if objetos else self._fallback_objetos('ninguna caja YOLO por encima de umbral')

    def _detectar_recipientes(self, imagen):
        """Busca recipientes de colores en la imagen."""
        if self.detector_color is None:
            return self._fallback_recipientes('detector de color no disponible')

        recipientes = []
        try:
            ancho = imagen.shape[1]
            recs_raw = self.detector_color.detectar_recipientes(imagen, area_minima=3000)
            colores_vistos = set()

            for r in recs_raw:
                if r['color'] in colores_vistos:
                    continue
                colores_vistos.add(r['color'])
                pos_rel = self.detector_color.posicion_relativa_en_imagen(r['centro'], ancho)
                recipientes.append(Recipiente(
                    color=r['color'], bbox=r['bbox'],
                    centro=r['centro'], posicion_relativa=pos_rel,
                ))

        except Exception as e:
            log.error(f"Error detectando recipientes: {e}")
            return self._fallback_recipientes('excepcion al detectar recipientes')

        if not recipientes:
            return self._fallback_recipientes('ningun area de color suficiente para recipiente')
        return recipientes

    def _deteccion_simulada(self):
        """Genera objetos simulados para pruebas sin camara."""
        log.info("Usando deteccion simulada")
        return [
            ObjetoDetectado('apple', 'rojo', 0.85, (100, 200, 200, 300), (150, 250), -0.3),
            ObjetoDetectado('bottle', 'azul', 0.90, (400, 150, 500, 350), (450, 250), 0.4),
        ]

    def _recipientes_por_defecto(self):
        """Posiciones fijas de recipientes cuando la deteccion visual falla.
        Asume recipientes colocados a izquierda, centro y derecha."""
        return [
            Recipiente('rojo', (0, 0, 100, 100), (50, 50), -0.7),
            Recipiente('azul', (200, 0, 300, 100), (250, 50), 0.0),
            Recipiente('verde', (400, 0, 500, 100), (450, 50), 0.7),
        ]

    # ------------------------------------------------------------------
    # PLANIFICACION
    # ------------------------------------------------------------------

    def _planificar(self, objetos, recipientes):
        """Genera lista de tareas: emparejar cada objeto con su recipiente
        del mismo color. Si no existe recipiente del color exacto, usa el
        mas cercano o un recipiente 'default'."""
        plan = []

        objetos_pendientes = [o for o in objetos if not o.agarrado and o.intentos_agarre < MAX_REINTENTOS_AGARRE]
        if not recipientes and not PERMITIR_DETECCION_SIMULADA:
            if objetos_pendientes:
                log.warning(
                    "Sin recipientes visibles (modo sin simulacion): no se genera plan para no depositar a ciegas."
                )
            return []

        objetos_pendientes.sort(key=lambda o: abs(o.posicion_relativa))

        for obj in objetos_pendientes:
            rec = self._buscar_recipiente(obj.color, recipientes)
            plan.append({
                'tipo': 'pick_and_place',
                'objeto': obj,
                'recipiente': rec,
            })

        log.info(f"Plan generado: {len(plan)} tareas")
        return plan

    def _buscar_recipiente(self, color_objeto, recipientes):
        """Encuentra el mejor recipiente para un color dado."""
        for rec in recipientes:
            if rec.color == color_objeto:
                return rec

        mapa_afinidad = {
            'rojo': ['naranja', 'amarillo'],
            'naranja': ['rojo', 'amarillo'],
            'amarillo': ['naranja', 'verde'],
            'verde': ['amarillo', 'azul'],
            'azul': ['morado', 'verde'],
            'morado': ['azul', 'rojo'],
        }
        afines = mapa_afinidad.get(color_objeto, [])
        for afin in afines:
            for rec in recipientes:
                if rec.color == afin:
                    return rec

        if recipientes:
            return recipientes[0]
        return Recipiente('default', (0, 0, 0, 0), (0, 0), 0.0)

    # ------------------------------------------------------------------
    # EJECUCION DE TAREAS
    # ------------------------------------------------------------------

    def _ejecutar_tarea(self, tarea):
        """Ejecuta una tarea pick_and_place completa con reintentos."""
        obj = tarea['objeto']
        rec = tarea['recipiente']
        exito = False

        log.info(f"\nTAREA: recoger '{obj.clase}' (color={obj.color}) -> recipiente '{rec.color}'")

        for intento in range(MAX_REINTENTOS_AGARRE):
            if self._detener:
                return

            obj.intentos_agarre += 1
            log.info(f"  Intento {intento + 1}/{MAX_REINTENTOS_AGARRE}")

            self._cambiar_estado(EstadoRobot.RECOGIENDO)
            pasos_obj = int(obj.posicion_relativa * self.FACTOR_PASOS_BASE)

            ajuste_vertical = 0.0
            if intento == 1:
                ajuste_vertical = 0.3
                log.info("  Estrategia: bajando un poco mas")
            elif intento == 2:
                ajuste_vertical = 0.5
                pasos_obj += 50 if obj.posicion_relativa > 0 else -50
                log.info("  Estrategia: bajando mas + micro-ajuste lateral")

            ok_pick = self.robot.secuencia_recoger(
                angulo_base_pasos=pasos_obj,
                tiempo_bajar=1.5 + ajuste_vertical,
                tiempo_cerrar=0.8 + (intento * 0.2),
                velocidad=VELOCIDAD_AUTONOMA,
            )

            if not ok_pick:
                log.warning(f"  Fallo en secuencia de recogida")
                self.estadisticas['agarres_fallidos'] += 1
                self.estadisticas['errores_recuperados'] += 1
                self.robot.posicion_home()
                time.sleep(1)
                continue

            time.sleep(0.5)
            agarre_ok = self.robot.verificar_agarre()

            if not agarre_ok and intento < MAX_REINTENTOS_AGARRE - 1:
                log.warning(f"  Agarre no confirmado, reintentando...")
                self.estadisticas['agarres_fallidos'] += 1
                self.robot.controlador_servo.mover_por_tiempo('gripper', 1, 0.5, 0.5)
                self.robot.posicion_home()
                time.sleep(1)
                continue

            self.estadisticas['agarres_exitosos'] += 1
            log.info(f"  Objeto agarrado exitosamente!")

            self._cambiar_estado(EstadoRobot.TRANSPORTANDO)
            pasos_rec = int(rec.posicion_relativa * self.FACTOR_PASOS_BASE)
            pasos_diferencia = pasos_rec - pasos_obj

            self._cambiar_estado(EstadoRobot.DEPOSITANDO)
            ok_place = self.robot.secuencia_soltar(
                angulo_base_pasos=pasos_diferencia,
                tiempo_bajar=1.2,
                velocidad=VELOCIDAD_AUTONOMA,
            )

            if ok_place:
                self.estadisticas['depositos_exitosos'] += 1
                rec.objetos_depositados += 1
                obj.agarrado = True
                exito = True
                log.info(f"  Deposito exitoso en recipiente '{rec.color}'!")
            else:
                log.warning(f"  Fallo en deposito")
                self.estadisticas['errores_recuperados'] += 1

            self.robot.posicion_home()
            time.sleep(0.5)
            break

        if not exito:
            log.warning(f"  SKIP: no se pudo recoger '{obj.clase}' despues de {MAX_REINTENTOS_AGARRE} intentos")
            self._registrar_evento('skip', obj.to_dict())

        self._registrar_evento('pick_place', {
            'objeto': obj.to_dict(), 'recipiente': rec.to_dict(),
            'exito': exito,
        })

    # ------------------------------------------------------------------
    # CONTROL DE ESTADO
    # ------------------------------------------------------------------

    def _cambiar_estado(self, nuevo_estado):
        with self._lock:
            anterior = self.estado
            self.estado = nuevo_estado
        if anterior != nuevo_estado:
            log.info(f"Estado: {anterior.name} -> {nuevo_estado.name}")

    def _esperar_si_pausado(self):
        while self._pausar and not self._detener:
            time.sleep(0.5)

    def pausar(self):
        self._pausar = True
        log.info("PAUSADO por usuario")

    def reanudar(self):
        self._pausar = False
        log.info("REANUDADO")

    def detener(self):
        self._detener = True
        self._pausar = False
        log.info("DETENCION solicitada")

    def obtener_estado(self):
        """Retorna snapshot del estado actual para la interfaz web."""
        with self._lock:
            return {
                'estado': self.estado.name,
                'objetos': [o.to_dict() for o in self.objetos],
                'recipientes': [r.to_dict() for r in self.recipientes],
                'estadisticas': self.estadisticas.copy(),
                'historial_reciente': self.historial[-10:],
                'ultimo_escaneo_ok': self._ultimo_escaneo_ok,
                'ultimo_escaneo_motivo': self._ultimo_escaneo_motivo,
            }

    def _registrar_evento(self, tipo, datos):
        evento = {
            'timestamp': datetime.now().isoformat(),
            'tipo': tipo,
            'datos': datos,
        }
        self.historial.append(evento)
        if len(self.historial) > 200:
            self.historial = self.historial[-100:]

    # ------------------------------------------------------------------
    # PUNTO DE ENTRADA
    # ------------------------------------------------------------------


def main():
    print("=" * 60)
    print("  BRAZO ROBOTICO AUTONOMO - Sistema de Pick & Place")
    print("  Deteccion por color + YOLO + Clasificacion inteligente")
    print("=" * 60)

    cerebro = CerebroAutonomo(habilitar_hardware=True)

    print("\nOpciones:")
    print("  [a] Iniciar modo AUTONOMO")
    print("  [e] Escanear una vez (prueba)")
    print("  [h] Ir a posicion HOME")
    print("  [q] Salir")

    while True:
        cmd = input("\n> ").strip().lower()
        if cmd == 'a':
            cerebro.ejecutar_ciclo_autonomo()
        elif cmd == 'e':
            objetos, recipientes = cerebro._escanear_entorno()
            print(f"\nResultado: {len(objetos)} objetos, {len(recipientes)} recipientes")
        elif cmd == 'h':
            cerebro.robot.posicion_home()
        elif cmd == 'q':
            cerebro.robot.cerrar()
            break
        else:
            print("Comando no reconocido")


if __name__ == '__main__':
    main()
