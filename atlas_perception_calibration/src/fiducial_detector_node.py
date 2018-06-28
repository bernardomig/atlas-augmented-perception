#!/usr/bin/env python

from math import sqrt

import rospy

from fiducial_msgs.msg import FiducialTransformArray

from geometry_msgs.msg import Vector3
from atlas_perception_msgs.msg import DetectedObjects3, DetectedObject3, BoundingBox3

class FiducialDetectorNode:

    def __init__(self, fidutials_topic, objects_topic):

        self._fid_sub = rospy.Subscriber(
            fidutials_topic,
            FiducialTransformArray,
            callback=self._callback,
            queue_size=10)

        self._obj_pub = rospy.Publisher(
            objects_topic,
            DetectedObjects3,
            queue_size=10)

    def _callback(self, fiducial_transforms):

        detected_objects = DetectedObjects3(
            header=fiducial_transforms.header,
            objects=[
                DetectedObject3(
                    id=transform.fiducial_id,
                    label='fiducial',
                    bbox=BoundingBox3(
                        transform=transform.transform,
                        size=Vector3(
                            x=sqrt(transform.fiducial_area) * 1e-3,
                            y=sqrt(transform.fiducial_area) * 1e-3,
                            z=sqrt(transform.fiducial_area) * 1e-3,
                        )
                    )
                )
                for transform in fiducial_transforms.transforms
            ],
        )

        self._obj_pub.publish(detected_objects)

if __name__ == '__main__':

    rospy.init_node('fiducial_object_transform')

    detector = FiducialDetectorNode('/fiducial_transforms', '/detected_objects_3d')

    rospy.spin()
