#!/usr/bin/env python

import rospy

from sensor_msgs.msg import CameraInfo

from image_geometry import PinholeCameraModel

from atlas_perception_msgs.msg import DetectedObject2, DetectedObject3, DetectedObjects2, DetectedObjects3, BoundingBox3

from geometry_msgs.msg import Vector3, Transform, Quaternion


class TransformerNode:
    """
    Transforms the 2d objects returned from the detector to 3d objects
    with the help the the frontal laser.
    """

    def __init__(self, objects2_topic, objects3_topic):
        camera_info = rospy.wait_for_message('camera_info', CameraInfo)

        self._camera_model = PinholeCameraModel()
        self._camera_model.fromCameraInfo(camera_info)

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

        z = w / self._camera_model.fx() / 0.25

        (x0, y0, _) = self._camera_model.projectPixelTo3dRay( (x, y) )
        (x1, y1, _) = self._camera_model.projectPixelTo3dRay( (x + w, x + h) )


        return DetectedObject3(
            id=object.id,
            label=object.label,
            bbox=BoundingBox3(
                transform=Transform(
                    translation=Vector3(x=0.5 * (x1+x0) / 0.4, y= 0.5 * (y1+y0) / 1.1, z=z),
                    rotation=Quaternion(x=0, y=0, z=0, w=1),
                ),
                size=Vector3(
                    x=0.9,
                    y=0.9,
                    z=0.9
                )
            )
        )


if __name__ == '__main__':
    rospy.init_node('atlas_perception_transformer_node')

    t = TransformerNode('detected_objects_2d', 'detected_objects_3d')

    rospy.spin()
