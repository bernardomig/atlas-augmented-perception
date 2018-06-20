#!/usr/bin/env python

import rospy

class Transformer:
    """
    Transforms the 2d objects returned from the detector to 3d objects
    with the help the the frontal laser.
    """

    def __init__(self, objects_topic, laser_topic, camera_topic, classes_properties):

        self._objects_sub = None
        self._laser_sub = None

        self._class_properties = classes_properties
    
    def laser_callback(self, laser):
        pass
    
    def objects_callback(self, objects):
        pass
    
    def transform_object_to_3d(self, obj, distance):
        pass
    
    def transform_laser_to_camera(self, laser):
        pass