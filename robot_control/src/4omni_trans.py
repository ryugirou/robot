#!/usr/bin/env python
### coding: UTF-8
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import Float32
class matrixcal:
    def __init__(self):
        rospy.init_node('matrixcal', anonymous=True,log_level=rospy.DEBUG)
        self.subscriber = rospy.Subscriber('cmd_vel', Twist, self.callback)
        self.motor0_cmd_vel_pub = rospy.Publisher('/wheel0', Float64, queue_size=10)
        self.motor1_cmd_vel_pub = rospy.Publisher('/wheel1', Float64, queue_size=10)
        self.motor2_cmd_vel_pub = rospy.Publisher('/wheel2', Float64, queue_size=10)
        self.motor3_cmd_vel_pub = rospy.Publisher('/wheel3', Float64, queue_size=10)

        self.betamotor0_cmd_vel_pub = rospy.Publisher('beta/wheel0', Float32, queue_size=10)
        self.betamotor1_cmd_vel_pub = rospy.Publisher('beta/wheel1', Float32, queue_size=10)
        self.betamotor2_cmd_vel_pub = rospy.Publisher('beta/wheel2', Float32, queue_size=10)
        self.betamotor3_cmd_vel_pub = rospy.Publisher('beta/wheel3', Float32, queue_size=10)

        self.R = rospy.get_param('~radius')
        self.r = rospy.get_param('~center_to_wheel')
        alpha = np.pi / 4 #angle
        cos = np.cos(alpha)
        sin = np.sin(alpha)
        self.A = np.array([[-cos,-sin,self.r],[sin,-cos,self.r],[cos,sin,self.r],[-sin,cos,self.r]])

    def callback(self,data):
        msg = Float64()
        msg_beta = Float32()
        v1 = np.array([data.linear.y,-data.linear.x,-data.angular.z]).reshape(-1,1)
        v2 = np.dot(self.A,v1).flatten() / self.R #2dimvector to 1dimvector
        msg.data = v2[0]
        self.motor0_cmd_vel_pub.publish(msg)
        msg_beta.data = v2[0]
        self.betamotor0_cmd_vel_pub.publish(msg_beta)
        
        msg.data = v2[1]
        self.motor1_cmd_vel_pub.publish(msg)
        msg_beta.data = v2[1]
        self.betamotor1_cmd_vel_pub.publish(msg_beta)

        msg.data = v2[2]
        self.motor2_cmd_vel_pub.publish(msg)
        msg_beta.data = v2[2]
        self.betamotor2_cmd_vel_pub.publish(msg_beta)

        msg.data = v2[3]
        self.motor3_cmd_vel_pub.publish(msg)
        msg_beta.data = v2[3]
        self.betamotor3_cmd_vel_pub.publish(msg_beta)

if __name__ == '__main__':
    try:
        instance = matrixcal()
        rospy.spin()        
    except rospy.ROSInterruptException:
        pass
