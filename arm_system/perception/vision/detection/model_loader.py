import os

from ultralytics import YOLO


class ModelLoader:
    """Carga modelo YOLO. Intenta NCNN (Pi) primero, luego yolo11s.pt."""
    def __init__(self):
        current_path = os.path.dirname(os.path.abspath(__file__))
        ncnn_path = os.path.join(current_path, 'models', 'yolo11s_ncnn_model')
        ncnn_bin = os.path.join(ncnn_path, 'model.ncnn.bin')

        if os.path.exists(ncnn_bin):
            self.model = YOLO(ncnn_path, task='detect')
        else:
            self.model = YOLO('yolo11s.pt', task='detect')

    def get_model(self) -> YOLO:
        return self.model
    