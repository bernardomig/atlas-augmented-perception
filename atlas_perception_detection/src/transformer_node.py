#!/usr/bin/env python

import rospy

from atlas_perception_msgs.msg import DetectedObject2, DetectedObject3, DetectedObjects2, DetectedObjects3, BoundingBox3

from geometry_msgs.msg import Vector3, Transform, Quaternion


class Transformer:
    """
    Transforms the 2d objects returned from the detector to 3d objects
    with the help the the frontal laser.
    """

    def __init__(self, objects2_topic, objects3_topic):

        self._objects_sub = rospy.Subscriber(
            objects2_topic, DetectedObjects2, self.objects_callback, queue_size=10)

        self._objects_pub = rospy.Publisher(
            objects3_topic, DetectedObjects3, queue_size=10)

    def objects_callback(self, objects):
        objects_in_3d = map(self.transform_object, objects.objects)

        objects_msg = DetectedObjects3(
            header=objects.header,
            objects=list(objects_in_3d)
        )

        self._objects_pub.publish(objects_msg)

    def transform_object(self, object):
        x = object.bbox.position.x
        y = object.bbox.position.y
        w = object.bbox.size.x
        h = object.bbox.size.y

        return DetectedObject3(
            id=object.id,
            label=object.label,
            bbox=BoundingBox3(
                transform=Transform(
                    translation=Vector3(x=x / 400 - 500, y=y / 400 - 500, z=1),
                    rotation=Quaternion(x=1, y=0, z=0, w=0),
                ),
                size=Vector3(
                    x=w / 400,
                    y=h/500,
                    z=0
                )
            )
        )


if __name__ == '__main__':
    rospy.init_node('atlas_perception_transformer_node')

    t = Transformer('detected_objects_2d', 'detected_objects_3d')

    rospy.spin()