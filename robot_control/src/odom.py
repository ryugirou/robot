#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from gazebo_msgs.srv import GetModelState
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

def euler_to_quaternion(euler):
    q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

def quaternion_to_euler(quaternion):
    e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    return Vector3(x=e[0], y=e[1], z=e[2])

if __name__ == '__main__':
    rospy.init_node('odom_broadcaster')
    pub = rospy.Publisher('/4omni/controller/odom',Odometry,queue_size=10)
    state_prox = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
    while not rospy.is_shutdown():
        r = Odometry()
        r.header = Header()
        r.header.stamp = rospy.Time.now()
        r.header.frame_id = "4omni/odom"
        r.child_frame_id ="4omni/base_link"
        rospy.wait_for_service('/gazebo/get_model_state')
        state = state_prox('4omni','world')
        euler = quaternion_to_euler(state.pose.orientation)
        euler.z *= 0.9
        r.pose.pose.orientation = euler_to_quaternion(euler)
        r.pose.pose.position = state.pose.position
        r.pose.covariance = [0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000.0]
        r.twist.twist = state.twist
        r.twist.covariance =[0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000.0]
        pub.publish(r)
        rospy.loginfo('Transform Published')
        rospy.Rate(200.0).sleep()
