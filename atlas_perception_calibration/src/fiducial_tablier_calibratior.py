#!/usr/bin/env python

from __future__ import print_function

import rospy

from tf import TransformListener, LookupException

class FiducialTablierCalibrator(object):

    def __init__(self, camera, fiducial):
        
        self._camera = camera
        self._fiducial = fiducial

        self._tf_list = TransformListener()

    def __call__(self):
        now = rospy.Time(0)
        self._tf_list.waitForTransform(self._camera, self._fiducial,
                              now, rospy.Duration(3.0))
        return self._tf_list.lookupTransform(
            self._camera,
            self._fiducial,
            now
        )

if __name__ == '__main__':
    rospy.init_node('fiducial_tablier_calibrator', anonymous=True)

    camera = rospy.get_param('~camera')
    fiducial = rospy.get_param('~fiducial')

    calib = FiducialTablierCalibrator(
        camera=camera,
        fiducial=fiducial)

    (trans, rot) = calib()

    print("translation = ", trans)
    print("rotation = ", rot)
