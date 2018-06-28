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
            stamp = timestamp,
            frame_id = frame_id,
        )
        
        new_objects = []

        for obj in objects.objects:
            new_objects.append(self.objects_msg_to_proto(header, obj))

        self._objects_deque.append(new_objects)

        print("received new objects!")
        print("queue size {}".format(len(self._objects_deque)))

    @staticmethod
    def objects_msg_to_proto(header, object):
        return DetectedObject(
            header=header,
            id=object.id,
            label=object.label,
            bbox=BoundingBox(
                transform=Transformation(
                    origin=Vector(
                        x=object.bbox.transform.translation.x,
                        y=object.bbox.transform.translation.y,
                        z=object.bbox.transform.translation.y),
                    orientation=Quaternion(
                        x=object.bbox.transform.rotation.x,
                        y=object.bbox.transform.rotation.y,
                        z=object.bbox.transform.rotation.z,
                        w=object.bbox.transform.rotation.w),
                    ),
                size=Size(
                    width=object.bbox.size.x,
                    heigth=object.bbox.size.y,
                    depth=object.bbox.size.z,
                )
            ),
        )


    def GetDetectedObjects(self, request, context):
        while True:
            try:
                obj = self._objects_deque.pop()
                print("sending object")
                yield DetectionServiceResponse(objects=obj)
            except IndexError:
                pass

if __name__ == '__main__':
    rospy.init_node('atlas_perception_interface')

    server = grpc.server(futures.ThreadPoolExecutor(max_workers=2))
    add_DetectionServiceServicer_to_server(DetectionService('detected_objects_3d'), server)
    server.add_insecure_port('127.0.0.1:5000')
    server.start()

    rospy.spin()