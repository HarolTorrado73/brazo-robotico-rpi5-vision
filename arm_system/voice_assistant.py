#!/usr/bin/env python3
"""
Asistente por voz: micrófono (USB o integrado) + TTS con timbre tipo robot.
Dependencias opcionales: ver requirements-voice.txt

- Linux / Raspberry Pi: espeak-ng (sudo apt install espeak-ng) para voz metálica.
- Sin espeak: pyttsx3 (menos “robot”, pero funcional).
- Reconocimiento: SpeechRecognition + Google (requiere red); falla de forma controlada si falta mic/PyAudio.
"""

from __future__ import annotations

import logging
import shutil
import subprocess
import threading
import unicodedata
from typing import Callable, Dict, List, Optional, Tuple

log = logging.getLogger(__name__)


def _normalizar_texto(s: str) -> str:
    s = unicodedata.normalize("NFD", (s or "").lower().strip())
    return "".join(c for c in s if unicodedata.category(c) != "Mn")


# Frases más específicas primero (evita ambigüedades).
_FRASES_A_ACCION: List[Tuple[str, str]] = [
    ("parada de emergencia", "emergencia"),
    ("emergencia", "emergencia"),
    ("calibrar los servos", "calibrar_servos"),
    ("calibrar servos", "calibrar_servos"),
    ("calibrar el color", "calibrar_color"),
    ("calibrar color", "calibrar_color"),
    ("reanudar", "reanudar"),
    ("continua", "reanudar"),
    ("continuar", "reanudar"),
    ("pausar", "pausar"),
    ("pausa", "pausar"),
    ("detener", "detener"),
    ("deten", "detener"),
    ("parar", "detener"),
    ("escanear el entorno", "escanear"),
    ("escanear", "escanear"),
    ("escanea", "escanear"),
    ("ir a casa", "home"),
    ("posicion home", "home"),
    ("ve a home", "home"),
    ("home", "home"),
    ("modo autonomo", "iniciar"),
    ("iniciar autonomo", "iniciar"),
    ("comenzar", "iniciar"),
    ("empieza", "iniciar"),
    ("iniciar", "iniciar"),
]


def texto_a_accion(texto: str) -> Optional[str]:
    """Devuelve clave de acción o None si no coincide ninguna frase."""
    n = _normalizar_texto(texto)
    if not n:
        return None
    for frase, accion in _FRASES_A_ACCION:
        if frase in n:
            return accion
    return None


class VozRobotica:
    """TTS no bloqueante en hilo de fondo; espeak-ng suena más “robot” que pyttsx3."""

    def __init__(
        self,
        idioma_espeak: str = "es",
        velocidad: int = 115,
        tono: int = 35,
        amplitud: int = 180,
    ):
        self._lock = threading.Lock()
        self._espeak = shutil.which("espeak-ng") or shutil.which("espeak")
        self._idioma = idioma_espeak
        self._velocidad = velocidad
        self._tono = tono
        self._amplitud = amplitud
        self._pyttsx3_engine = None

    def hablar(self, texto: str) -> None:
        t = (texto or "").strip()
        if not t:
            return

        def _run() -> None:
            try:
                if self._espeak:
                    subprocess.run(
                        [
                            self._espeak,
                            "-v",
                            self._idioma,
                            "-s",
                            str(self._velocidad),
                            "-p",
                            str(self._tono),
                            "-a",
                            str(self._amplitud),
                            t,
                        ],
                        check=False,
                        timeout=90,
                        capture_output=True,
                    )
                    return
                import pyttsx3

                with self._lock:
                    if self._pyttsx3_engine is None:
                        eng = pyttsx3.init()
                        eng.setProperty("rate", 130)
                        self._pyttsx3_engine = eng
                    self._pyttsx3_engine.say(t)
                    self._pyttsx3_engine.runAndWait()
            except Exception as e:
                log.warning("TTS no disponible o falló: %s", e)

        threading.Thread(target=_run, daemon=True).start()


class AsistenteVoz:
    """
    Bucle en segundo plano: escucha, reconoce, ejecuta callback(accion).
    """

    def __init__(
        self,
        acciones: Dict[str, Callable[[], None]],
        voz: Optional[VozRobotica] = None,
        idioma_google: str = "es-ES",
        timeout_escucha: float = 4.0,
        max_frase_seg: float = 10.0,
        mic_device_index: Optional[int] = None,
    ):
        self._acciones = acciones
        self._voz = voz or VozRobotica()
        self._idioma_google = idioma_google
        self._timeout_escucha = timeout_escucha
        self._max_frase_seg = max_frase_seg
        self._mic_device_index = mic_device_index
        self._stop = threading.Event()
        self._hilo: Optional[threading.Thread] = None

    @property
    def voz(self) -> VozRobotica:
        return self._voz

    def iniciar(self) -> bool:
        if self._hilo and self._hilo.is_alive():
            return True
        try:
            import speech_recognition as sr  # noqa: F401
        except ImportError:
            log.error("Instala dependencias de voz: pip install -r requirements-voice.txt")
            return False
        self._stop.clear()
        self._hilo = threading.Thread(target=self._bucle, daemon=True)
        self._hilo.start()
        if self._mic_device_index is not None:
            log.info(
                "Asistente de voz activo (Google Speech API, idioma %s, mic PyAudio #%s)",
                self._idioma_google,
                self._mic_device_index,
            )
        else:
            log.info(
                "Asistente de voz activo (Google Speech API, idioma %s, mic predeterminado)",
                self._idioma_google,
            )
        self._voz.hablar("Sistema de voz listo.")
        return True

    def detener(self) -> None:
        self._stop.set()

    def _bucle(self) -> None:
        import speech_recognition as sr

        r = sr.Recognizer()
        r.dynamic_energy_threshold = True
        mic_kw = {}
        if self._mic_device_index is not None:
            mic_kw["device_index"] = self._mic_device_index

        while not self._stop.is_set():
            try:
                with sr.Microphone(**mic_kw) as source:
                    r.adjust_for_ambient_noise(source, duration=0.35)
                    audio = r.listen(
                        source,
                        timeout=self._timeout_escucha,
                        phrase_time_limit=self._max_frase_seg,
                    )
            except sr.WaitTimeoutError:
                continue
            except OSError as e:
                log.error("Micrófono no disponible: %s", e)
                self._voz.hablar("Error de micrófono.")
                threading.Event().wait(5)
                continue
            except Exception as e:
                log.debug("Escucha: %s", e)
                continue

            try:
                texto = r.recognize_google(audio, language=self._idioma_google)
            except sr.UnknownValueError:
                continue
            except sr.RequestError as e:
                log.warning("Reconocimiento (red/servicio): %s", e)
                continue

            log.info("Voz reconocida: %s", texto)
            accion = texto_a_accion(texto)
            if not accion:
                self._voz.hablar("Comando no reconocido.")
                continue
            fn = self._acciones.get(accion)
            if not fn:
                self._voz.hablar("Acción no implementada.")
                continue
            try:
                fn()
            except Exception as e:
                log.exception("Error ejecutando acción de voz %s: %s", accion, e)
                self._voz.hablar("Error al ejecutar la orden.")


def voz_disponible() -> Tuple[bool, str]:
    """Comprueba si hay TTS razonable (espeak o pyttsx3)."""
    if shutil.which("espeak-ng") or shutil.which("espeak"):
        return True, "espeak"
    try:
        import pyttsx3  # noqa: F401

        return True, "pyttsx3"
    except ImportError:
        return False, "sin TTS (instala espeak-ng o pip install pyttsx3)"
