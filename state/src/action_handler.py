#!/usr/bin/env python
### coding: UTF-8
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8

class Actions:
  def __init__(self):
    self.cmd_publisher = rospy.Publisher('cmd',UInt8,queue_size=10)
    self.vel_publisher = rospy.Publisher('cmd_vel',Twist,queue_size=1)
    self.goal_publisher = rospy.Publisher('goal',UInt8,queue_size=10)
    self.result_subscriber = rospy.Subscriber('result',UInt8, self.__resultCallback)
    self.__result = 0
    self.max_lin = 0.5
    self.max_ang = 0.5

  def enable(self):
    cmd_msg = UInt8()
    cmd_msg.data = 1
    self.cmd_publisher.publish(cmd_msg)
    rospy.loginfo("enable")

  def disable(self):
    cmd_msg = UInt8()
    cmd_msg.data = 0
    self.cmd_publisher.publish(cmd_msg)
    rospy.loginfo("disable")

  def teleop(self,vel_x,vel_y,vel_z):
      norm = math.sqrt(vel_x**2+vel_y**2)
      if norm > 1.0:
        vel_x = vel_x / norm
        vel_y = vel_y / norm
      else:
        vel_x = vel_x
        vel_y = vel_y

      vel_x *= self.max_lin
      vel_y *= self.max_lin
      vel_z *= self.max_ang

      vel_msg = Twist()
      vel_msg.linear.x = vel_x
      vel_msg.linear.y = vel_y
      vel_msg.angular.z = vel_z
      self.vel_publisher.publish(vel_msg)

  def send_goal(self,trajectory):
    goal_msg = UInt8()
    goal_msg.data = trajectory
    self.goal_publisher.publish(goal_msg)

  def __resultCallback(self,data):
    self.__result = data.data

  def getResult(self):
    if self.__result:
      self.__result = 0
      return True
    else:
      return False