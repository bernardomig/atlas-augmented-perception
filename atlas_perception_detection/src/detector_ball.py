
from detector import BaseDetector

import rospy
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

import numpy as np
import cv2 as cv


class BallDetector(BaseDetector):
    """A stub detector that always outputs the same prediction.
    Should be used as a way to test the system.
    """

    def __init__(self):
        pass

    def load_model(self, model): pass

    def predict(self, image):
        img = cv.GaussianBlur(image, (11, 11), 0.5)
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

        lower_red = np.array([0, 100, 100])
        upper_red = np.array([50, 255, 255])

        mask = cv.inRange(hsv, lower_red, upper_red)

        mask = cv.erode(mask, None, iterations=2)
        mask = cv.dilate(mask, None, iterations=2)

        _, contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        cnt = map(lambda cnt: (len(cnt), cnt), contours)
        cnt = filter(lambda (l, _): l > 500, cnt)
        cnt = sorted(cnt, lambda cnt: cnt[0])
        cnt = map(lambda cnt: cnt[1], cnt)
        cnt = map(cv.convexHull, cnt)
        bounding_rect = map(cv.boundingRect, cnt)
        keypoints = [
            { 'pt': [x, y], 'size': [w, h]}
            for (x, y, w, h) in bounding_rect
        ]

        return [
            {'label': 'ball',
                'bbox': {'x': kp['pt'][0], 'y': kp['pt'][1], 'w': kp['size'][0], 'h': kp['size'][1]}, }
            for kp in keypoints
        ]
