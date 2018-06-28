#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from atlas_perception_msgs.msg import BoundingBox2, DetectedObject2, DetectedObjects2
from cv_bridge import CvBridge
import numpy as np

def _to_detected_object_msg(id, detected_object):
    """Converts the output of the detector node to a DetectedObjects2 message."""
    bbox = detected_object['bbox']
    return DetectedObject2(
        id=id,
        label=detected_object['label'],
        bbox=BoundingBox2(
            position=Vector3(x=bbox['x'], y=bbox['y']),
            size=Vector3(x=bbox['w'], y=bbox['h']),
        )
    )

class DetectorNode:
    """The DetectorNode listens to messages incomming in the image topic, and
    outputs the predictions to detected_object topic.
    The message, when received, is converted to a numpy array and then passed
    to the detector class.
    """

    def __init__(self, detector, image_topic, detected_object_topic):
        self._image_sub = rospy.Subscriber(
            name=image_topic,
            data_class=Image,
            callback=self.callback,
            queue_size=10)

        self._detected_objects_pub = rospy.Publisher(
            name=detected_object_topic,
            data_class=DetectedObjects2,
            queue_size=10)

        self._detector = detector

        self._cv_bridge = CvBridge()

    def callback(self, image):
        nd_array = self._cv_bridge.imgmsg_to_cv2(image)
        header = image.header
        detected_object_arr = self._detector.predict(nd_array)

        detected_objects_msg = DetectedObjects2(
            header=header,
            objects=list(map(lambda a: _to_detected_object_msg(a[0],a[1]), enumerate(detected_object_arr)))
        )

        print(detected_objects_msg)

        self._detected_objects_pub.publish(detected_objects_msg)

#
# This is runs a test node using the TestDetector as the detector and uses the topics:
# - 'image' as the image topic
# - 'detected_objects_2d' as the detected objects topic
#
if __name__ == '__main__':

    import sys

    from detector_test import TestDetector
    from detector_ball import BallDetector

    rospy.init_node("detector_node")

    node = DetectorNode(BallDetector(), 'image', 'detected_objects')

    rospy.spin()
