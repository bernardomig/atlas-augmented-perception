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

class DetectionService(DetectionServiceServicer):

    def GetDetectedObjects(self, request, context):
        for i in range(100):
            timestamp = Timestamp()
            timestamp.GetCurrentTime()
            yield DetectionServiceResponse(objects=[
                DetectedObject(
                    id=1,
                    label="person",
                    header=Header(frame_id="some_frame", stamp=timestamp),
                    bbox=BoundingBox(
                        transform=Transformation(
                            origin=Vector(x=random(), y=random(), z=random()),
                            orientation=Quaternion(x=random(), y=random(), z=random(), w=random())),
                        size=Size(width=random(), heigth=random(), depth=random()),
                    )
                )
            ])
            time.sleep(0.3)

if __name__ == '__main__':
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=2))
    add_DetectionServiceServicer_to_server(DetectionService(), server)
    server.add_insecure_port('127.0.0.1:5000')
    server.start()

    try:
        while True:
            time.sleep(10)
    except KeyboardInterrupt:
        server.stop(0)
