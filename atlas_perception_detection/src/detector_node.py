#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from atlas_perception_msgs.msg import BoundingBox2, DetectedObject2, DetectedObjects2
from cv_bridge import CvBridge
import numpy as np

def _to_detected_object_msg(id, detected_object):
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


if __name__ == '__main__':

    from detector import TestDetector

    detector = TestDetector()

    rospy.init_node("detector_node_debug")

    node = DetectorNode(detector, 'image', 'detected_object_2d')

    rospy.spin()
    