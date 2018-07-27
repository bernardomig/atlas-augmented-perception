#!/usr/bin/env python

from math import sqrt

import rospy

from fiducial_msgs.msg import FiducialTransformArray

from geometry_msgs.msg import Vector3, Quaternion

from tf import TransformBroadcaster
from tf import transformations as t

from geometry_msgs.msg import TransformStamped

class FiducialTFPublisher(object):

    def __init__(self, fiducial_topic, fiducial_id, frame_id, invert=False):

        self._fiducial_id = fiducial_id
        self._frame_id = frame_id
        self._invert = invert
        self._fid_sub = rospy.Subscriber(
            fiducial_topic,
            FiducialTransformArray,
            callback=self._callback,
            queue_size=10)

        self._tf_brd = TransformBroadcaster()
    
    def _callback(self, fiducial_transforms):
        header = fiducial_transforms.header
        child_frame_id = self._frame_id

        if self._invert:
            child_frame_id = header.frame_id
            header.frame_id = self._frame_id

        obj = filter(lambda a: a.fiducial_id == self._fiducial_id, fiducial_transforms.transforms)

        if len(obj) > 0:
            obj = obj[0]

            if self._invert:
                trans = obj.transform.translation
                rot = obj.transform.rotation
                trans = [trans.x, trans.y, trans.z]
                rot = [rot.x, rot.y, rot.z, rot.w]
                m_trans = t.translation_matrix(trans)
                m_rot = t.quaternion_matrix(rot)
                transform = t.concatenate_matrices(m_trans, m_rot)
                inverse_transform = t.inverse_matrix(transform)
                trans = t.translation_from_matrix(inverse_transform)
                rot = t.quaternion_from_matrix(inverse_transform)
                obj.transform.translation = Vector3(x=trans[0], y=trans[1], z=trans[2])
                obj.transform.rotation = Quaternion(x=rot[0], y=rot[1], z=rot[2], w=rot[3])


            self._tf_brd.sendTransformMessage(
                TransformStamped(
                    header=header,
                    child_frame_id=child_frame_id,
                    transform=obj.transform)
                )

if __name__ == '__main__':

    rospy.init_node('fiducial_tf_publisher', anonymous=True)

    frame_id = rospy.get_param('~frame_id')
    fiducial_id = rospy.get_param('~fiducial_id')
    invert_tf = rospy.get_param('~invert_tf', default=False)

    pub = FiducialTFPublisher(
        fiducial_topic='fiducial_transforms',
        fiducial_id=fiducial_id,
        frame_id=frame_id,
        invert=invert_tf
    )

    rospy.spin()