#/usr/bin/env python2

import time
from random import random
from google.protobuf.timestamp_pb2 import Timestamp

from messages.detectedobjects_pb2 import DetectedObjects
from messages.detectedobject_pb2 import DetectedObject
from messages.header_pb2 import Header
from messages.boundingbox_pb2 import BoundingBox
from messages.vector_pb2 import Vector
from messages.quaternion_pb2 import Quaternion
from messages.transformation_pb2 import Transformation
from messages.size_pb2 import Size

from flask import Flask

def generate_object(id):
	return DetectedObject(
		id=id,
		label="person",
		boundingBox=BoundingBox(
			transform=Transformation(
				origin=Vector(x=random(), y=random(), z=random()),
				orientation=Quaternion(x=random(), y=random(), z=random(), w=random())),
			size=Size(width=random(), heigth=random(), depth=random()),
		)
	)

def generate_objects():
	timestamp = Timestamp()
	timestamp.GetCurrentTime()
	header=Header(frame_id="some_frame", stamp=timestamp)
	detectedObjects = [generate_object(i) for i in range(2)]
	return DetectedObjects(
		header=header,
		detectedObjects=detectedObjects,
	)

app = Flask(__name__)

@app.route('/')
def index():
	return generate_objects().SerializeToString()
