#!/usr/bin/env python

import sys
import signal
from collections import deque
from concurrent import futures
import time
from random import random

from google.protobuf.json_format import MessageToJson

from google.protobuf.timestamp_pb2 import Timestamp

from messages.detectedobjects_pb2 import DetectedObjects
from messages.detectedobject_pb2 import DetectedObject
from messages.header_pb2 import Header
from messages.boundingbox_pb2 import BoundingBox
from messages.vector_pb2 import Vector
from messages.quaternion_pb2 import Quaternion
from messages.transformation_pb2 import Transformation
from messages.size_pb2 import Size

import rospy
from atlas_perception_msgs.msg import DetectedObjects3

from flask import Flask

class DetectionService:

    def __init__(self, objects_topic):

        self._last_objects = None

        self._objects_subscriber = rospy.Subscriber(
            objects_topic,
            DetectedObjects3,
            callback=self.objects_callback,
            queue_size=10)

    def objects_callback(self, objects):
        timestamp = Timestamp()
        timestamp.seconds = objects.header.stamp.secs
        timestamp.nanos = objects.header.stamp.nsecs
        frame_id = objects.header.frame_id
        header = Header(
            stamp = timestamp,
            frame_id = frame_id,
        )

        new_objects = [ self.objects_msg_to_proto(obj) for obj in objects.objects ]

        self._last_objects = DetectedObjects(
            header=header,
            detectedObjects=new_objects)

    @staticmethod
    def objects_msg_to_proto(object):
        return DetectedObject(
            id=object.id,
            label=object.label,
            boundingBox=BoundingBox(
                transform=Transformation(
                    translation=Vector(
                        x=object.bbox.transform.translation.x,
                        y=-object.bbox.transform.translation.y,
                        z=object.bbox.transform.translation.y),
                    rotation=Quaternion(
                        x=object.bbox.transform.rotation.x,
                        y=object.bbox.transform.rotation.y,
                        z=object.bbox.transform.rotation.z,
                        w=object.bbox.transform.rotation.w),
                    ),
                size=Size(
                    width=object.bbox.size.x,
                    height=object.bbox.size.y,
                    depth=object.bbox.size.z,
                )
            ),
        )

    def get_all_objects(self):
        return self._last_objects

if __name__ == '__main__':

    rospy.init_node('atlas_perception_interface')

    ds = DetectionService('detected_objects_3d')

    app = Flask('atlas_perception_interface')

    @app.route('/')
    def index():
        return ds.get_all_objects().SerializeToString()

    @app.route('/debug')
    def debug():
        return MessageToJson(ds.get_all_objects())

    signal.signal(signal.SIGINT, lambda: sys.exit(0))

    app.run(debug=False)

