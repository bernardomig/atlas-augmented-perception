
from detector import BaseDetector

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np

class TestDetector(BaseDetector):
    """A stub detector that always outputs the same prediction.
    Should be used as a way to test the system.
    """

    def __init__(self, image_topic):
        self._image_sub = rospy.Subscriber(
            image_topic,
            Image,
            callback=self.image_callback,
            queue_size=10)

    def load_model(self, model): pass

    def image_callback(self, image):
        pass

    def predict(self, image):
        return [
            {'label': 'ball',
             'bbox': {'x': 0, 'y': 0, 'w': 100, 'h': 100}, },
        ]
