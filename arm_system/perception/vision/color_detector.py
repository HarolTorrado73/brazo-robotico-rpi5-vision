"""
Detector de colores HSV adaptativo para clasificar objetos y localizar recipientes.
Trabaja en conjunto con YOLO: YOLO detecta el objeto, este modulo analiza
su color dominante para decidir en que recipiente depositarlo.

Mejoras sobre version anterior:
- CLAHE para normalizar iluminacion antes del analisis HSV
- Sistema de scoring multi-criterio (porcentaje + distancia HSV + consistencia espacial)
- Mapeo YOLO->color como fuente primaria, HSV como fallback
- Calibracion de offsets HSV por iluminacion
"""
import cv2
import numpy as np
import logging as log

RANGOS_COLOR_HSV = {
    'rojo': [
        {'h_min': 0, 'h_max': 10, 's_min': 70, 's_max': 255, 'v_min': 50, 'v_max': 255},
        {'h_min': 170, 'h_max': 180, 's_min': 70, 's_max': 255, 'v_min': 50, 'v_max': 255},
    ],
    'azul': [
        {'h_min': 100, 'h_max': 130, 's_min': 50, 's_max': 255, 'v_min': 50, 'v_max': 255},
    ],
    'verde': [
        {'h_min': 35, 'h_max': 85, 's_min': 40, 's_max': 255, 'v_min': 40, 'v_max': 255},
    ],
    'amarillo': [
        {'h_min': 18, 'h_max': 35, 's_min': 50, 's_max': 255, 'v_min': 80, 'v_max': 255},
    ],
    'naranja': [
        {'h_min': 10, 'h_max': 22, 's_min': 100, 's_max': 255, 'v_min': 80, 'v_max': 255},
    ],
    'morado': [
        {'h_min': 125, 'h_max': 155, 's_min': 40, 's_max': 255, 'v_min': 40, 'v_max': 255},
    ],
}

YOLO_CLASE_A_COLOR = {
    'apple': 'rojo',
    'orange': 'naranja',
    'banana': 'amarillo',
    'broccoli': 'verde',
    'carrot': 'naranja',
    'sports ball': 'naranja',
    'bottle': 'verde',
    'cup': 'azul',
    'teddy bear': 'amarillo',
}


class DetectorColor:
    """Analiza regiones de imagen para determinar color dominante
    con normalizacion de iluminacion y scoring multi-criterio."""

    def __init__(self, rangos_personalizados=None):
        self.rangos = rangos_personalizados or RANGOS_COLOR_HSV
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        self.h_offset = 0
        self.s_offset = 0
        self.v_offset = 0
        self.clase_yolo_a_color = dict(YOLO_CLASE_A_COLOR)
        try:
            from config_sistema import YOLO_LAB_CLASE_A_COLOR

            self.clase_yolo_a_color.update(YOLO_LAB_CLASE_A_COLOR)
        except ImportError:
            pass

    def _normalizar_iluminacion(self, imagen_bgr):
        """Aplica CLAHE al canal L de LAB para ecualizar iluminacion."""
        lab = cv2.cvtColor(imagen_bgr, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        l = self.clahe.apply(l)
        lab = cv2.merge([l, a, b])
        return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    def _aplicar_offsets_hsv(self, hsv):
        """Aplica offsets de calibracion al espacio HSV."""
        if self.h_offset == 0 and self.s_offset == 0 and self.v_offset == 0:
            return hsv
        hsv = hsv.astype(np.int16)
        hsv[:, :, 0] = np.clip(hsv[:, :, 0] + self.h_offset, 0, 179)
        hsv[:, :, 1] = np.clip(hsv[:, :, 1] + self.s_offset, 0, 255)
        hsv[:, :, 2] = np.clip(hsv[:, :, 2] + self.v_offset, 0, 255)
        return hsv.astype(np.uint8)

    def color_dominante_region(self, imagen_bgr, bbox, clase_yolo=None):
        """Dado un bounding box (x1,y1,x2,y2) en una imagen BGR,
        retorna el nombre del color dominante y su porcentaje de cobertura.

        Si clase_yolo se proporciona y tiene un mapeo conocido con confianza > 0.6,
        se usa como fuente primaria."""
        if clase_yolo and clase_yolo in self.clase_yolo_a_color:
            color_yolo = self.clase_yolo_a_color[clase_yolo]
            color_hsv, pct_hsv = self._analizar_hsv_region(imagen_bgr, bbox)
            if color_hsv == color_yolo:
                return color_yolo, max(pct_hsv, 0.5)
            return color_yolo, 0.5

        return self._analizar_hsv_region(imagen_bgr, bbox)

    def _analizar_hsv_region(self, imagen_bgr, bbox):
        """Analisis HSV puro con CLAHE y scoring multi-criterio."""
        x1, y1, x2, y2 = map(int, bbox)
        h, w = imagen_bgr.shape[:2]
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)

        if x2 - x1 < 5 or y2 - y1 < 5:
            return 'desconocido', 0.0

        recorte = imagen_bgr[y1:y2, x1:x2]
        recorte = self._normalizar_iluminacion(recorte)
        hsv = cv2.cvtColor(recorte, cv2.COLOR_BGR2HSV)
        hsv = self._aplicar_offsets_hsv(hsv)
        total_pixeles = hsv.shape[0] * hsv.shape[1]

        mejor_color = 'desconocido'
        mejor_score = 0.0

        for nombre_color, rangos_lista in self.rangos.items():
            mascara_total = np.zeros(hsv.shape[:2], dtype=np.uint8)
            centros_h = []

            for rango in rangos_lista:
                inferior = np.array([rango['h_min'], rango['s_min'], rango['v_min']])
                superior = np.array([rango['h_max'], rango['s_max'], rango['v_max']])
                mascara = cv2.inRange(hsv, inferior, superior)
                mascara_total = cv2.bitwise_or(mascara_total, mascara)
                centros_h.append((rango['h_min'] + rango['h_max']) / 2.0)

            pixeles_color = cv2.countNonZero(mascara_total)
            porcentaje = pixeles_color / total_pixeles

            if porcentaje < 0.03:
                continue

            score_porcentaje = min(porcentaje / 0.5, 1.0) * 0.5

            if pixeles_color > 0:
                h_pixeles = hsv[:, :, 0][mascara_total > 0]
                h_medio = np.mean(h_pixeles)
                mejor_dist = min(abs(h_medio - c) for c in centros_h)
                dist_normalizada = 1.0 - min(mejor_dist / 30.0, 1.0)
            else:
                dist_normalizada = 0.0
            score_distancia = dist_normalizada * 0.3

            if pixeles_color > 10:
                num_labels, _ = cv2.connectedComponents(mascara_total)
                if num_labels > 1:
                    score_consistencia = min(pixeles_color / ((num_labels - 1) * 50), 1.0) * 0.2
                else:
                    score_consistencia = 0.0
            else:
                score_consistencia = 0.0

            score_total = score_porcentaje + score_distancia + score_consistencia

            if score_total > mejor_score:
                mejor_score = score_total
                mejor_color = nombre_color

        if mejor_score < 0.05:
            return 'desconocido', mejor_score

        return mejor_color, mejor_score

    def calibrar_iluminacion(self, imagen_bgr):
        """Calibra offsets HSV analizando la region central de la imagen,
        asumiendo que deberia tener valores neutros de S y V medios.
        Retorna los offsets calculados."""
        h, w = imagen_bgr.shape[:2]
        cx, cy = w // 2, h // 2
        margen = min(w, h) // 6
        recorte = imagen_bgr[cy - margen:cy + margen, cx - margen:cx + margen]

        hsv = cv2.cvtColor(recorte, cv2.COLOR_BGR2HSV)
        v_mean = np.mean(hsv[:, :, 2])
        s_mean = np.mean(hsv[:, :, 1])

        self.v_offset = int(128 - v_mean)
        self.s_offset = int(max(0, 80 - s_mean))
        self.h_offset = 0

        log.info(f"[Color] Calibracion iluminacion: V_offset={self.v_offset} "
                 f"S_offset={self.s_offset} (V_mean={v_mean:.0f} S_mean={s_mean:.0f})")
        return {'h_offset': self.h_offset, 's_offset': self.s_offset, 'v_offset': self.v_offset}

    def detectar_recipientes(self, imagen_bgr, area_minima=2000):
        """Busca rectangulos grandes de colores solidos (recipientes) en la imagen.
        Retorna lista de dict con: color, bbox, centro, area."""
        imagen_norm = self._normalizar_iluminacion(imagen_bgr)
        hsv = cv2.cvtColor(imagen_norm, cv2.COLOR_BGR2HSV)
        hsv = self._aplicar_offsets_hsv(hsv)
        recipientes = []

        for nombre_color, rangos_lista in self.rangos.items():
            mascara_total = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for rango in rangos_lista:
                inferior = np.array([rango['h_min'], rango['s_min'], rango['v_min']])
                superior = np.array([rango['h_max'], rango['s_max'], rango['v_max']])
                mascara_total = cv2.bitwise_or(mascara_total, cv2.inRange(hsv, inferior, superior))

            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
            mascara_total = cv2.morphologyEx(mascara_total, cv2.MORPH_CLOSE, kernel)
            mascara_total = cv2.morphologyEx(mascara_total, cv2.MORPH_OPEN, kernel)

            contornos, _ = cv2.findContours(mascara_total, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contorno in contornos:
                area = cv2.contourArea(contorno)
                if area < area_minima:
                    continue

                x, y, w, h = cv2.boundingRect(contorno)
                aspecto = w / h if h > 0 else 0
                if 0.3 < aspecto < 3.5:
                    centro = (x + w // 2, y + h // 2)
                    recipientes.append({
                        'color': nombre_color,
                        'bbox': (x, y, x + w, y + h),
                        'centro': centro,
                        'area': area,
                    })

        recipientes.sort(key=lambda r: r['area'], reverse=True)
        return recipientes

    def posicion_relativa_en_imagen(self, centro, ancho_imagen):
        """Convierte posicion X del centro de un objeto en la imagen
        a una estimacion de -1.0 (extremo izquierdo) a 1.0 (extremo derecho)."""
        cx = centro[0]
        return (cx / ancho_imagen - 0.5) * 2.0

    def dibujar_resultados(self, imagen, objetos_detectados, recipientes):
        """Dibuja bboxes de objetos y recipientes sobre la imagen."""
        colores_bgr = {
            'rojo': (0, 0, 255), 'azul': (255, 0, 0), 'verde': (0, 255, 0),
            'amarillo': (0, 255, 255), 'naranja': (0, 140, 255),
            'morado': (180, 0, 255), 'desconocido': (128, 128, 128),
        }
        vis = imagen.copy()

        for obj in objetos_detectados:
            x1, y1, x2, y2 = map(int, obj['bbox'])
            color_bgr = colores_bgr.get(obj.get('color', ''), (255, 255, 255))
            cv2.rectangle(vis, (x1, y1), (x2, y2), color_bgr, 2)
            etiqueta = f"{obj.get('clase', '?')} [{obj.get('color', '?')}] {obj.get('confianza', 0):.0%}"
            cv2.putText(vis, etiqueta, (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 2)

        for rec in recipientes:
            x1, y1, x2, y2 = rec['bbox']
            color_bgr = colores_bgr.get(rec['color'], (200, 200, 200))
            cv2.rectangle(vis, (x1, y1), (x2, y2), color_bgr, 3)
            cv2.putText(vis, f"REC:{rec['color']}", (x1, y2 + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2)

        return vis
