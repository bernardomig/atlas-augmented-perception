#!/usr/bin/env python

import rospy

from atlas_perception_msgs.msg import DetectedObjects3

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3, Quaternion


class Object3DVisualizerNode:

    def __init__(self, objects_topic):

        self._obj_sub = rospy.Subscriber(
            objects_topic, DetectedObjects3, callback=self._callback, queue_size=10)

        self._marker_pub = rospy.Publisher(
            'visualization_marker_array', MarkerArray, queue_size=10
        )

    def _callback(self, objects):
        header = objects.header

        markers = [
            Marker(
                header=header,
                ns='detected_objects_3d',
                id=obj.id,
                type=Marker.SPHERE,
                action=0,
                pose=Pose(
                    position=obj.bbox.transform.translation,
                    orientation=obj.bbox.transform.rotation,
                ),
                scale=obj.bbox.size,
                color=ColorRGBA(0, 1, 0, 1),
            )
            for obj in objects.objects
        ]

        self._marker_pub.publish(markers)

if __name__ == '__main__':
    rospy.init_node('object3d_visualizer_node')

    ov = Object3DVisualizerNode('detected_objects_3d')

    rospy.spin()
