#!/usr/bin/env python

from __future__ import print_function

import rospy
from geometry_msgs.msg import Vector3, Quaternion, Transform

from tf import TransformListener

from atlas_perception_msgs.msg import DetectedObjects3, DetectedObject3

from tf import transformations as t

class ObjectsChangeReferencial:

    def __init__(self, input_topic, output_topic, target_referencial):
        self._target_referencial = target_referencial

        self._input_sub = rospy.Subscriber(input_topic, DetectedObjects3, callback=self.callback, queue_size=10)
        self._output_pub = rospy.Publisher(output_topic, DetectedObjects3, queue_size=10)

        self._tf_list = TransformListener()

    def change_ref(self, frame_id, stamp, transform):
        trans = transform.translation
        rot = transform.rotation
        trans = [trans.x, trans.y, trans.z]
        rot = [rot.x, rot.y, rot.z, rot.w]
        m_trans = t.translation_matrix(trans)
        m_rot = t.quaternion_matrix(rot)
        transform1 = t.concatenate_matrices(m_trans, m_rot)
        trans, rot = self._tf_list.lookupTransform(self._target_referencial, frame_id, stamp)
        m_trans = t.translation_matrix(trans)
        m_rot = t.quaternion_matrix(rot)
        transform2 = t.concatenate_matrices(m_trans, m_rot)
        transform = transform2.dot(transform1)
        print('t1 + t2 = ', transform)
        trans = t.translation_from_matrix(transform)
        rot = t.quaternion_from_matrix(transform)
        return Transform(
            translation=Vector3(x=trans[0], y=trans[1], z=trans[2]),
            rotation=Quaternion(x=rot[0], y=rot[1], z=rot[2], w=rot[3]),
        )


    def callback(self, msg):

        frame_id = msg.header.frame_id
        stamp = msg.header.stamp
        for id, obj in enumerate(msg.objects):
            msg.objects[id].bbox.transform = self.change_ref(frame_id, stamp, obj.bbox.transform)

        msg.header.frame_id = self._target_referencial

        self._output_pub.publish(msg)

if __name__ == '__main__':

    rospy.init_node('object_change_ref', anonymous=True)

    frame_id = rospy.get_param('~target_referencial')

    pub = ObjectsChangeReferencial(
        input_topic="input_objects",
        output_topic="output_objects",
        target_referencial=frame_id)

    rospy.spin()
