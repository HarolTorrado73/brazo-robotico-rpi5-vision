"""
CameraManager - Captura de imagen para Raspberry Pi 5 con Arducam CSI.
Prioridad: Picamera2 -> rpicam-still -> libcamera-still -> OpenCV
"""
import os
import time
import subprocess
import logging as log
import cv2

log.basicConfig(level=log.INFO, format="%(asctime)s [%(levelname)s] %(message)s")


class CameraManager:
    """Gestor de camara para Raspberry Pi 5 con Arducam CSI."""

    def __init__(self, width: int = 640, height: int = 480, flip: bool = True):
        self.flip = flip
        self.width = width
        self.height = height
        self.metodo = None
        self.picam2 = None
        self.cap = None
        self._cmd_captura = None

        if self._init_picamera2():
            self.metodo = 'picamera2'
        elif self._init_rpicam_still():
            self.metodo = 'rpicam-still'
        elif self._init_libcamera():
            self.metodo = 'libcamera-still'
        elif self._init_opencv():
            self.metodo = 'opencv'
        else:
            log.error("CAMARA: Ningun metodo de captura funciono")

        if self.metodo:
            log.info(f"CAMARA: Inicializada con '{self.metodo}' ({self.width}x{self.height})")

    def _init_picamera2(self):
        try:
            from picamera2 import Picamera2
            cam = Picamera2()
            config = cam.create_still_configuration(
                main={"size": (self.width, self.height), "format": "RGB888"}
            )
            cam.configure(config)
            cam.start()
            time.sleep(2)
            test = cam.capture_array()
            if test is not None and test.size > 0:
                self.picam2 = cam
                return True
            cam.stop()
        except (ImportError, Exception):
            pass
        return False

    def _init_rpicam_still(self):
        return self._probar_comando('rpicam-still')

    def _init_libcamera(self):
        return self._probar_comando('libcamera-still')

    def _probar_comando(self, cmd_name):
        try:
            r = subprocess.run([cmd_name, '--list-cameras'],
                              capture_output=True, text=True, timeout=10)
            if r.returncode == 0 and 'Available' in r.stdout:
                self._cmd_captura = cmd_name
                return True
        except (FileNotFoundError, subprocess.TimeoutExpired):
            pass
        return False

    def _init_opencv(self):
        try:
            cap = cv2.VideoCapture(0)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                ret, frame = cap.read()
                if ret and frame is not None:
                    self.cap = cap
                    return True
                cap.release()
        except Exception:
            pass
        return False

    def _flip_image(self, img):
        if self.flip and img is not None:
            return cv2.rotate(img, cv2.ROTATE_180)
        return img

    def capture_image(self, save: bool = True):
        """Captura imagen. Retorna (imagen, ruta) o (None, None)."""
        image = None
        try:
            if self.metodo == 'picamera2' and self.picam2:
                arr = self.picam2.capture_array()
                if arr is not None and arr.size > 0:
                    image = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)

            elif self.metodo in ('rpicam-still', 'libcamera-still') and self._cmd_captura:
                image = self._captura_still()

            elif self.metodo == 'opencv' and self.cap:
                for _ in range(3):
                    self.cap.grab()
                ret, image = self.cap.read()
                if not ret:
                    image = None

            if image is None:
                return None, None

            image = self._flip_image(image)

            if save:
                base = os.path.dirname(os.path.abspath(__file__))
                path = f"{base}/../objects_images/{time.strftime('%Y%m%d-%H%M%S')}.jpg"
                os.makedirs(os.path.dirname(path), exist_ok=True)
                cv2.imwrite(path, image)
                return image, path
            return image, None
        except Exception as e:
            log.error(f"CAMARA: Error {e}")
            return None, None

    def _captura_still(self):
        temp = "/tmp/brazo_capture.jpg"
        try:
            if os.path.exists(temp):
                os.remove(temp)
        except OSError:
            pass
        try:
            cmd = [self._cmd_captura, '-o', temp, '-n',
                   '--width', str(self.width), '--height', str(self.height)]
            subprocess.run(cmd, capture_output=True, timeout=15)
            if os.path.exists(temp):
                img = cv2.imread(temp)
                try:
                    os.remove(temp)
                except OSError:
                    pass
                return img
        except subprocess.TimeoutExpired:
            log.warning("CAMARA: rpicam-still timeout")
        return None

    def __del__(self):
        try:
            if self.picam2:
                self.picam2.stop()
        except Exception:
            pass
        try:
            if self.cap:
                self.cap.release()
        except Exception:
            pass
