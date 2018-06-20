#!/usr/bin/env python

from collections import deque

import grpc
from concurrent import futures
import time
from random import random
from google.protobuf.timestamp_pb2 import Timestamp

from messages.detectionservice_pb2_grpc import DetectionServiceServicer, add_DetectionServiceServicer_to_server
from messages.detectionservice_pb2 import DetectionServiceRequest, DetectionServiceResponse
from messages.detectedobject_pb2 import DetectedObject
from messages.header_pb2 import Header
from messages.boundingbox_pb2 import BoundingBox
from messages.vector_pb2 import Vector
from messages.quaternion_pb2 import Quaternion
from messages.transformation_pb2 import Transformation
from messages.size_pb2 import Size

import rospy
from atlas_perception_msgs.msg import DetectedObjects3


class DetectionService(DetectionServiceServicer):

    def __init__(self, objects_topic):
        self._objects_deque = deque(maxlen=10)

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
            timestmap = timestamp,
            frame_id = frame_id,
        )
        
        new_objects = []

        for obj in objects.objects:
            new_objects.append(self.objects_msg_to_proto(header, obj))

        self._objects_deque.append(new_objects)

    @staticmethod
    def objects_msg_to_proto(header, object):
        return DetectedObject(
            header=header,
            id=object.id,
            label=object.label,
            bbox=BoundingBox(
                transform=Transformation(
                    origin=Vector(
                        x=object.bbox.transform.origin.x,
                        y=object.bbox.transform.origin.y,
                        z=object.bbox.transform.origin.y),
                    orientation=Quaternion(
                        x=object.bbox.transform.orientation.x,
                        y=object.bbox.transform.orientation.y,
                        z=object.bbox.transform.orientation.z,
                        w=object.bbox.transform.orientation.w),
                    ),
                size=Size(
                    width=object.bbox.size.x,
                    height=object.bbox.size.w,
                    depth=object.bbox.size.z,
                )
            ),
        )


    def GetDetectedObjects(self, request, context):
        while True:
            try:
                obj = self._objects_deque.pop()
                yield obj
            except IndexError:
                pass

if __name__ == '__main__':
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=2))
    add_DetectionServiceServicer_to_server(DetectionService('objects_3d'), server)
    server.add_insecure_port('127.0.0.1:5000')
    server.start()

    try:
        while True:
            time.sleep(10)
    except KeyboardInterrupt:
        server.stop(0)
