#!/usr/bin/env python

import rospy

from tf import TransformListener, LookupException

class FiducialTablierCalibrator(object):

    def __init__(self, camera, fiducial):
        
        self._camera = camera
        self._fiducial = fiducial

        self._tf_list = TransformListener()

    def __call__(self):
        return self._tf_list.lookupTransform(
            self._camera,
            self._fiducial,
            rospy.Time(0)
        )

if __name__ == '__main__':
    rospy.init_node('fiducial_tablier_calibrator', anonymous=True)

    camera = rospy.get_param('camera')
    fiducial = rospy.get_param('fiducial')

    calib = FiducialTablierCalibrator(
        camera=camera,
        fiducial=fiducial)
    
    print(calib())