import time
import logging as log
from control.robot_controller import ControladorRobotico
from config_sistema import STEPPER_HABILITADO
from safety.safe_controller import SafeController

log.basicConfig(level=log.INFO, format="%(asctime)s - %(levelname)s - %(message)s")


class Robot:
    def __init__(self):
        self.robot = ControladorRobotico(habilitar_stepper=STEPPER_HABILITADO)
        # SafeController para control manual interactivo (ángulos, servo_config.json)
        self._safe = SafeController()
        self.serial_manager = None

        try:
            from communication.serial_manager import CommunicationManager
            self.serial_manager = CommunicationManager()
            if not self.serial_manager.connect():
                log.warning("No se pudo conectar con el puerto serial - modo sin hardware")
                self.serial_manager = None
        except Exception as e:
            log.warning(f"Error inicializando comunicacion serial: {e} - modo sin hardware")
            self.serial_manager = None

        self.scan_results = []

        self.placement_zones = {
            'apple': {'angle': 90, 'distance': 200},
            'orange': {'angle': 180, 'distance': 200},
            'bottle': {'angle': 45, 'distance': 200},
            'default': {'angle': 270, 'distance': 200},
        }
        
    # --- MENU ---
    def main_menu_loop(self):
        running = True
        while running:
            print("\n=== MENU PRINCIPAL ===")
            print(" [a] MODO AUTONOMO (pick & place automatico)")
            print(" [w] Interfaz WEB autonoma")
            print(" [n] Escanear objetos")
            print(" [p] Pick & place (manual)")
            print(" [m] Control manual")
            print(" [h] Posicion HOME")
            print(" [q] Salir")

            user_input = input("> ").strip().lower()

            if user_input == 'a':
                self._iniciar_modo_autonomo()

            elif user_input == 'w':
                self._iniciar_web_autonoma()

            elif user_input == 'n':
                self.handle_scan_command()

            elif user_input == 'p':
                self.handle_pick_place_command()

            elif user_input == 'm':
                self.manual_control_menu()

            elif user_input == 'h':
                self.robot.posicion_home()

            elif user_input == 'q':
                running = False

            else:
                print("Comando no reconocido")

            time.sleep(0.5)

    def _iniciar_modo_autonomo(self):
        """Lanzar el cerebro autonomo."""
        from autonomous_brain import CerebroAutonomo
        cerebro = CerebroAutonomo(habilitar_hardware=True)
        cerebro.ejecutar_ciclo_autonomo()

    def _iniciar_web_autonoma(self):
        """Lanzar servidor web autonomo."""
        print("Iniciando interfaz web en http://0.0.0.0:5000")
        print("Presiona Ctrl+C para detener")
        import subprocess
        import sys
        subprocess.run([sys.executable, 'autonomous_web.py'])
            
    # --- SCAN ---
    def handle_scan_command(self):
        """scan command"""

        from perception.vision.camera.main import CameraManager
        from perception.vision.detection.main import DetectionModel

        self.scan_results = []

        try:
            camera = CameraManager()
            detector = DetectionModel()
        except Exception as e:
            log.error(f"Error inicializando componentes de vision: {e}")
            self._simulate_detection()
            return

        log.info("scanning in progress...")

        try:
            image_path = camera.capture_image()
            if not image_path:
                log.warning("failed to capture image - usando modo simulado")
                self._simulate_detection()
                return
        except Exception as e:
            log.warning(f"Error capturando imagen: {e} - usando modo simulado")
            self._simulate_detection()
            return

        import cv2
        image = cv2.imread(image_path)

        results, names = detector.inference(image)

        for result in results:
            boxes = result.boxes
            for box in boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                xyxy = box.xyxy[0].cpu().numpy()
                x1, y1, x2, y2 = xyxy
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2

                angle = (center_x / image.shape[1]) * 180
                distance = 200

                data = {
                    'class': names[cls],
                    'confidence': conf,
                    'angle': angle,
                    'distance': distance,
                    'image_path': image_path
                }
                self._scan_callback(data)

        self.process_scan_results()
        
    def _scan_callback(self, data):
        if data.get('class'):
            self._update_object_registry(data)
            
    def _update_object_registry(self, data: dict):
        """update object registry"""
        try:
            self.scan_results.append({
                'position': {
                    'angle': data.get('angle', 0),
                    'distance': data.get('distance', 0)
                },
                'detection':{
                    'class': data.get('class', 'default'),
                    'confidence': data.get('confidence', 0.0),
                    'image': data.get('image_path', '')
                },
                'placement_zone': self._get_placement_zones(data.get('class', 'default'))
            })
        except Exception as e:
            log.error(f"error updating registry: {str(e)}")
        
    def _get_placement_zones(self, object_class: str):
        return self.placement_zones.get(object_class.lower(), 
                                        self.placement_zones['default'])          
        
    def process_scan_results(self):
        """process scan data"""
        if not self.scan_results:
            log.warning("scanning completed without object detection")
            return
            
        log.info(f"\n=== objects scanned: ({len(self.scan_results)}) ===")
        processed_list = []
        for i, obj in enumerate(self.scan_results, start=1):
            angle = obj['position']['angle']
            distance = obj['position']['distance']
            obj_class = obj['detection']['class']
            confidence = obj['detection']['confidence']
            zone = obj['placement_zone']

            item = {
                'index': i,
                'center_angle': angle,
                'distance': distance,
                'class': obj_class,
                'confidence': confidence,
                'placement_zone': zone
            }
            processed_list.append(item)

            log.info(f"Obj {i} -> angle: {angle}, distance: {distance}mm, class: {obj_class}, conf: {confidence:.2f}")

        self.scan_results = processed_list

    def manual_control_menu(self):
        """Menu de control manual del brazo via SafeController (ángulos, 10° por paso)."""
        print("\n=== CONTROL MANUAL (SafeController) ===")
        print("Cada comando mueve 10° en la dirección indicada:")
        print(" [s+] hombro +    [s-] hombro -")
        print(" [e+] codo +      [e-] codo -")
        print(" [w+] muñeca +    [w-] muñeca -")
        print(" [g+] pinza abrir [g-] pinza cerrar")
        print(" [b+] base +      [b-] base -")
        print(" [h] HOME  [em] emergency stop  [re] reset emergency  [q] volver")

        while True:
            sim_tag = " [SIM]" if self._safe.is_simulation else ""
            emerg_tag = " [EMERGENCY]" if self._safe.is_emergency else ""
            cmd = input(f"manual{sim_tag}{emerg_tag}> ").strip().lower()
            if cmd == 'q':
                break
            elif cmd == 'h':
                self._safe.go_home()
            elif cmd == 'em':
                self._safe.emergency_stop()
                print("*** Emergency stop activado. Usa 're' para reanudar. ***")
            elif cmd == 're':
                self._safe.reset_emergency()
                print("Emergency stop reiniciado.")
            else:
                self._ejecutar_comando_manual(cmd)

    def _ejecutar_comando_manual(self, cmd):
        """
        Parsea y ejecuta comandos manuales pasando por SafeController.
        Cada pulsación mueve 10° en la dirección indicada.
        """
        PASO_DEG = 10.0
        mapa = {
            's+': ('shoulder', +PASO_DEG),
            's-': ('shoulder', -PASO_DEG),
            'e+': ('elbow',    +PASO_DEG),
            'e-': ('elbow',    -PASO_DEG),
            'w+': ('wrist',    +PASO_DEG),
            'w-': ('wrist',    -PASO_DEG),
            'g+': ('gripper',  +PASO_DEG),
            'g-': ('gripper',  -PASO_DEG),
            'b+': ('base',     +PASO_DEG),
            'b-': ('base',     -PASO_DEG),
        }
        if cmd not in mapa:
            print("Comando no reconocido")
            return

        articulacion, delta = mapa[cmd]

        if self._safe.is_emergency:
            log.error("[Manual] Emergency stop activo. Usa 'e' en el menú para reiniciarlo.")
            return

        ok = self._safe.move_relative(articulacion, delta)
        if ok:
            log.info(
                "[Manual] %s %+.0f° → %.1f°",
                articulacion, delta, self._safe.get_angle(articulacion),
            )
        else:
            log.warning("[Manual] Movimiento rechazado por SafeController: %s %+.0f°", articulacion, delta)

    def _simulate_detection(self):
        """Simular deteccion de objetos para modo demo"""
        log.info("Modo simulado: Generando deteccion de ejemplo")

        simulated_objects = [
            {
                'class': 'apple',
                'confidence': 0.85,
                'angle': 45,
                'distance': 180,
                'image_path': 'simulated'
            },
            {
                'class': 'bottle',
                'confidence': 0.92,
                'angle': 135,
                'distance': 220,
                'image_path': 'simulated'
            }
        ]

        for data in simulated_objects:
            self._scan_callback(data)

        self.process_scan_results()
    
    # --- PICK & PLACE ---
    def handle_pick_place_command(self):
        """pick & place command"""
        if not self.scan_results:
            log.warning("1. first scanning the enviroment (option 'n')")
            return

        selected_object = self.select_object_interactively()
        if not selected_object:
            return

        log.info(f"\ninit pick & place to object: {selected_object['index']}:")
        log.info(f"angle: {selected_object['center_angle']}")
        log.info(f"distance: {selected_object['distance']} mm")
        
        if self.execute_pick_sequence(selected_object):
            log.info(f"pick completed!")
            if self.execute_place_sequence(selected_object):
                log.info(f"pick and place completed!")
                
    def select_object_interactively(self):
        """interface for object selection"""
        print("\n=== OBJECTS DETECTED LIST ===")
        for o in self.scan_results:
            i = o['index']
            print(f"[{i}] angle={o['center_angle']} dist={o['distance']}mm class={o['class']} conf={o['confidence']:.2f}")
        print("[0] cancelar")
        
        try:
            selection = int(input("\nselect the object you want to take: "))
            if selection == 0:
                print("operation canceled")
                return {}
            
            return next((x for x in self.scan_results if x['index'] == selection), {})
        
        except ValueError:
            print("invalid input")
            return {}
        
    def execute_pick_sequence(self, target_object: dict) -> bool:
        """Secuencia de recogida usando movimientos por tiempo."""
        try:
            pasos_base = int((target_object['center_angle'] / 180.0) * 400)
            ok = self.robot.secuencia_recoger(
                angulo_base_pasos=pasos_base,
                tiempo_bajar=1.5,
                tiempo_cerrar=0.8,
            )
            return ok
        except Exception as e:
            log.error(f"Error en secuencia pick: {e}")
            self.robot._posicion_segura()
            return False

    def execute_place_sequence(self, target_object: dict):
        """Secuencia de deposito usando movimientos por tiempo."""
        try:
            zone_params = target_object['placement_zone']
            pasos_zona = int((zone_params['angle'] / 180.0) * 400)
            pasos_actual = int((target_object['center_angle'] / 180.0) * 400)
            pasos_diff = pasos_zona - pasos_actual

            ok = self.robot.secuencia_soltar(
                angulo_base_pasos=pasos_diff,
                tiempo_bajar=1.2,
            )
            return ok
        except Exception as e:
            log.error(f"Error en secuencia place: {e}")
            self.robot._posicion_segura()
            return False

    def handle_movement_failure(self):
        """Protocolo de seguridad ante fallos."""
        log.error("Ejecutando protocolo de seguridad")
        try:
            self.robot._posicion_segura()
        except Exception as e:
            log.error(f"Error en seguridad: {e}")
            self.robot.cerrar()
            exit(1)
            
            
    def run(self):
        try:
            log.info("starting robot controller")
            print("\n" + "=" * 60)
            print("  Consola = menu basico. Para checklist, guia /docs y voz (si la")
            print("  activas en config_sistema.py) usa la interfaz web:")
            print("    cd arm_system  &&  python autonomous_web.py")
            print("    Navegador: http://<IP_DE_TU_RASPBERRY>:5000")
            print("=" * 60 + "\n")
            self.main_menu_loop()

        except KeyboardInterrupt:
            log.info("Programa interrumpido por el usuario.")
        finally:
            log.info("Cerrando controlador del robot.")
            self.robot.cerrar()
            self._safe.close()
            if self.serial_manager:
                self.serial_manager.close()


if __name__ == '__main__':
    robot = Robot()
    robot.run()
