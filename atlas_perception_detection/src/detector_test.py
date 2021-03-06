
from detector import BaseDetector

class TestDetector(BaseDetector):
    """A stub detector that always outputs the same prediction.
    Should be used as a way to test the system.
    """

    def __init__(self): pass

    def load_model(self, model): pass

    def predict(self, image):
        return [
            {   'label': 'person',
                'bbox': {'x': 0, 'y': 0, 'w': 100, 'h': 100},   },
            {   'label': 'horse',
                'bbox': {'x': 100, 'y': 0, 'w': 100, 'h': 100}, },
            {   'label': 'bicycle',
                'bbox': {'x': 0, 'y': 100, 'w': 100, 'h': 100}, },
            {   'label': 'car',
                'bbox': {'x': 300, 'y': 0, 'w': 100, 'h': 100}, },
        ]
