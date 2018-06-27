#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from atlas_perception_msgs.msg import DetectedObjects2
from cv_bridge import CvBridge

import cv2 as cv


class ObjectVisualizerNode:

    def __init__(self, image_topic, objects_topic, debug_topic):
        self._image_sub = rospy.Subscriber(
            image_topic, Image, callback=self._image_callback, queue_size=10)
        
        self._objects_sub = rospy.Subscriber(
            objects_topic, DetectedObjects2, callback=self._objects_callback, queue_size=10)

        self._cv_brige = CvBridge()

        self._last_image = None

        self._image_pub = rospy.Publisher(debug_topic, Image, queue_size=10)

    def _image_callback(self, image):
        img = self._cv_brige.imgmsg_to_cv2(image)
        self._last_image = img


    def _objects_callback(self, objects):
        if self._last_image is None: return
        
        img = self._last_image

        for obj in objects.objects:
            x0, y0 = int(obj.bbox.position.x), int(obj.bbox.position.y)
            w, h = int(obj.bbox.size.x), int(obj.bbox.size.y)
            x1, y1 = x0 + w, y0 + h
            cv.rectangle(img, (x0, y0), (x1, y1), (0, 255, 0), 3)
            label = "{}:{}".format(obj.id, obj.label)
            cv.putText(img, label, (x0+3, y1-3), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        self._image_pub.publish(self._cv_brige.cv2_to_imgmsg(img, encoding="bgr8"))


if __name__ == '__main__':
    rospy.init_node('object_visualizer_node')

    obv = ObjectVisualizerNode(
        'camera/image_raw', 'detected_objects_2d', 'image_with_objects')

    rospy.spin()
