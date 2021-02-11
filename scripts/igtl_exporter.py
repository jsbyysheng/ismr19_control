#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import math
import tf
import geometry_msgs.msg
import numpy as np
from ros_igtl_bridge.msg import igtlstring
from ros_igtl_bridge.msg import igtltransform
from geometry_msgs.msg import Transform
from urdf_parser_py.urdf import URDF

robot = URDF.from_parameter_server()

def push_transform(pub, name, trans, rot):
    transform = Transform()
    transform.translation.x = trans[0] * 1000
    transform.translation.y = trans[1] * 1000
    transform.translation.z = trans[2] * 1000
    transform.rotation.x = rot[0]
    transform.rotation.y = rot[1]
    transform.rotation.z = rot[2]    
    transform.rotation.w = rot[3]
    
    transmsg = igtltransform()
    transmsg.name = name
    transmsg.transform = transform

    pub.publish(transmsg)

def set_visual_origin(tf_Ts):
    tf1_trans, tf1_rot, tf2_trans, tf2_rot = tf_Ts[0], tf_Ts[1], tf_Ts[2], tf_Ts[3]

    trans_mat1 = tf.transformations.translation_matrix(tf1_trans)
    rot_mat1   = tf.transformations.quaternion_matrix(tf1_rot)
    mat1 = np.dot(trans_mat1, rot_mat1)

    trans_mat2 = tf.transformations.translation_matrix(tf2_trans)
    rot_mat2   = tf.transformations.quaternion_matrix(tf2_rot)
    mat2 = np.dot(trans_mat2, rot_mat2)

    mat3 = np.dot(mat1, mat2)
    trans3 = tf.transformations.translation_from_matrix(mat3)
    rot3 = tf.transformations.quaternion_from_matrix(mat3)

    return trans3, rot3
    

def igtl_exporter():

    base_link_name = 'base_link'
    links_name = [
        'shoulder_link',
        'upper_arm_link',
        'forearm_link', 
        'wrist_1_link', 
        'wrist_2_link', 
        'wrist_3_link', 
        'tool0',
        'needle_holder',
        'needle'
    ]

    rospy.init_node('igtl_exporter', anonymous=True)

    pub_igtl_transform_out = rospy.Publisher('IGTL_TRANSFORM_OUT', igtltransform, queue_size=10)    

    listener = tf.TransformListener()
    rate = rospy.Rate(10) # 10hz

    tf_trans = dict()

    for link in robot.links:
        if link.name in links_name:
            # trans, rot_Q, Visual_Origin_trans, Visual_Origin_rot_Q,
            tf_trans[link.name] = [
                [0.0, 0.0, 0.0], 
                [0.0, 0.0, 0.0, 1.0], 
                [0.0, 0.0, 0.0], 
                [0.0, 0.0, 0.0, 1.0]
            ]
            if (link.visual is not None) and (link.visual.origin is not None):
                tf_trans[link.name][2] = link.visual.origin.position
                rot = link.visual.origin.rotation
                tf_trans[link.name][3] = tf.transformations.quaternion_from_euler(rot[0], rot[1], rot[2]).tolist()


    listener.waitForTransform("/" + base_link_name, "/" + links_name[0], rospy.Time(), rospy.Duration(4.0))
    
    while not rospy.is_shutdown():
        try:
            for key in tf_trans:
                (tf_trans[key][0],tf_trans[key][1]) = listener.lookupTransform('/' + base_link_name, '/' + key, rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(e)

        for key in tf_trans:
            trans, rot = set_visual_origin(tf_trans[key])
            push_transform(pub_igtl_transform_out, key, trans, rot)

        rate.sleep()

if __name__ == '__main__':

    igtl_exporter()


