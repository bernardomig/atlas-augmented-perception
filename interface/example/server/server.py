import grpc
from concurrent import futures
import time
from random import random

from messages.DetectionService_pb2_grpc import DetectionServiceServicer, add_DetectionServiceServicer_to_server
from messages.DetectionService_pb2 import DetectionServiceRequest, DetectionServiceResponse
from messages.DetectedObject_pb2 import DetectedObject
from messages.BoundingBox3_pb2 import BoundingBox3
from messages.Position3_pb2 import Position3
from messages.Size3_pb2 import Size3

class DetectionService(DetectionServiceServicer):

    def GetDetectedObjects(self, request, context):
        for i in range(100):
            yield DetectionServiceResponse(objects=[
                DetectedObject(
                    id=1,
                    label="person",
                    bbox=BoundingBox3(
                        position=Position3(x=random(), y=random(), z=random()),
                        size=Size3(width=random(), heigth=random(), depth=random()),
                    )
                )
            ])
            time.sleep(0.3)

if __name__ == '__main__':
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=2))
    add_DetectionServiceServicer_to_server(DetectionService(), server)
    server.add_insecure_port('[::]:5000')
    server.start()

    try:
        while True:
            time.sleep(10)
    except KeyboardInterrupt:
        server.stop(0)
