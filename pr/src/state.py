#!/usr/bin/env python
### coding: UTF-8
import math
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class joy:
  def __init__(self):
    self.ButtonA=rospy.get_param("~ButtonA")
    self.ButtonB=rospy.get_param("~ButtonB")
    self.ButtonX=rospy.get_param("~ButtonX")
    self.ButtonY=rospy.get_param("~ButtonY")
    self.ButtonLB=rospy.get_param("~ButtonLB")
    self.ButtonRB=rospy.get_param("~ButtonRB")
    self.ButtonSelect=rospy.get_param("~ButtonSelect")
    self.ButtonStart=rospy.get_param("~ButtonStart")
    self.ButtonLeftThumb=rospy.get_param("~ButtonLeftThumb")
    self.ButtonRightThumb=rospy.get_param("~ButtonRightThumb")

    self.AxisDPadX=rospy.get_param("~AxisDPadX")
    self.AxisDPadY=rospy.get_param("~AxisDPadY")
    self.AxisLeftThumbX=rospy.get_param("~AxisLeftThumbX")
    self.AxisLeftThumbY=rospy.get_param("~AxisLeftThumbY")
    self.AxisRightThumbX=rospy.get_param("~AxisRightThumbX")
    self.AxisRightThumbY=rospy.get_param("~AxisRightThumbY")
    self.AxisLeftTrigger=rospy.get_param("~AxisLeftTrigger")
    self.AxisRightTrigger=rospy.get_param("~AxisRightTrigger")

    self.max_lin = rospy.get_param("~max_lin")
    self.max_ang = rospy.get_param("~max_ang")

    self.vel_x = 0
    self.vel_y = 0
    self.vel_z = 0
    self.enable = 0
    self.disable = 0

    self.A = 0
    self.B = 0
    self.X = 0
    self.Y = 0
    self.Select = 0
    self.Start = 0
    self.LT = 0 
    self.counter = 0
    self.last = 1
    self.last_status = [0]*15
    self.next = 0

    self.joy_subscriber = rospy.Subscriber('joy',Joy, self.joyCallback)
    self.solenoid_publisher = rospy.Publisher('solenoid',UInt8, queue_size=10)
    self.slide_publisher = rospy.Publisher('slide',Float64, queue_size=10)
    self.launcher_publisher = rospy.Publisher('launcher',Float64, queue_size=10)
    self.arm_publisher = rospy.Publisher('arm',Float64, queue_size=10)
    self.vel_publisher = rospy.Publisher('cmd_vel',Twist,queue_size=10)
    self.cmd_publisher = rospy.Publisher('cmd',UInt8,queue_size=10)
    self.pusshed=0

    rospy.Timer(rospy.Duration(0.1), self.timerCallback)

  def save_last_stat(self,data):
    self.last_status[self.ButtonA] = data.buttons[self.ButtonA]
    self.last_status[self.ButtonB] = data.buttons[self.ButtonB]
    self.last_status[self.ButtonX] = data.buttons[self.ButtonX]
    self.last_status[self.ButtonY] = data.buttons[self.ButtonY]
    self.last_status[self.ButtonLB] = data.buttons[self.ButtonLB]
    self.last_status[self.ButtonRB] = data.buttons[self.ButtonRB]
    self.last_status[self.ButtonSelect] = data.buttons[self.ButtonSelect]
    self.last_status[self.ButtonStart] = data.buttons[self.ButtonStart]
    self.last_status[self.ButtonLeftThumb] = data.buttons[self.ButtonLeftThumb]
    self.last_status[self.ButtonRightThumb] = data.buttons[self.ButtonRightThumb]
  def joyCallback(self,data):
    self.vel_x = data.axes[self.AxisLeftThumbY]
    self.vel_y = data.axes[self.AxisLeftThumbX]
    self.vel_z = data.axes[self.AxisRightThumbX]

    self.enable = data.buttons[self.ButtonLB]
    self.disable = data.buttons[self.ButtonRB]

    if self.last_status[self.ButtonA] == 0 and data.buttons[self.ButtonA] == 1:
      self.A = 1

    if self.last_status[self.ButtonB] == 0 and data.buttons[self.ButtonB] == 1:
      self.B = 1

    if self.last_status[self.ButtonX] == 0 and data.buttons[self.ButtonX] == 1:
      self.X = 1

    if self.last_status[self.ButtonY] == 0 and data.buttons[self.ButtonY] == 1:
      self.Y = 1

    if self.last_status[self.ButtonSelect] == 0 and data.buttons[self.ButtonSelect] == 1:
      self.Select = 1

    if self.last_status[self.ButtonStart] == 0 and data.buttons[self.ButtonStart] == 1:
      self.Start = 1

    if self.last_status[self.ButtonLeftThumb] == 0 and data.buttons[self.ButtonLeftThumb] == 1:
      self.LT = 1

    if self.last == 0 and data.buttons[self.ButtonA]==1:
      rospy.logwarn('pushed')
      self.next=1

    self.last = data.buttons[self.ButtonA]
    self.save_last_stat(data)

  def timerCallback(self,event):
    if 1:
      norm = math.sqrt(self.vel_x**2+self.vel_y**2)
      if norm > 1.0:
        vel_x = self.vel_x / norm
        vel_y = self.vel_y / norm
      else:
        vel_x = self.vel_x
        vel_y = self.vel_y
      vel_z = self.vel_z

      vel_x *= self.max_lin
      vel_y *= self.max_lin
      vel_z *= self.max_ang

      vel_msg = Twist()
      vel_msg.linear.x = vel_x
      vel_msg.linear.y = vel_y
      vel_msg.angular.z = vel_z
      self.vel_publisher.publish(vel_msg)

    if self.enable==1:
      cmd_msg = UInt8()
      cmd_msg.data = 1
      self.cmd_publisher.publish(cmd_msg)
      self.enable = 0

    if self.disable==1:
      cmd_msg = UInt8()
      cmd_msg.data = 0
      self.cmd_publisher.publish(cmd_msg)
      self.disable = 0
    
    if self.A == 1:
      self.A = 0
      arm_msg = Float64()
      arm_msg.data = 1.8#1.75
      self.arm_publisher.publish(arm_msg)
    
    elif self.B == 1:
      self.B = 0
      solenoid_msg = UInt8()
      solenoid_msg.data = 0xff
      self.solenoid_publisher.publish(solenoid_msg)

    elif self.X == 1:
      self.X = 0
      arm_msg = Float64()
      arm_msg.data = 1.3
      self.arm_publisher.publish(arm_msg)

    elif self.Y == 1:
      self.Y = 0
      slide_msg = Float64()
      slide_msg.data = 0.66
      self.slide_publisher.publish(slide_msg)

      launcher_msg = Float64()
      launcher_msg.data = 250
      self.launcher_publisher.publish(launcher_msg)

    elif self.Select == 1:
      self.Select = 0
      arm_msg = Float64()
      arm_msg.data = 0.7
      self.arm_publisher.publish(arm_msg)

    elif self.Start == 1:
      self.Start = 0
      slide_msg = Float64()
      slide_msg.data = 0.0
      self.slide_publisher.publish(slide_msg)

      launcher_msg = Float64()
      launcher_msg.data = 0.0
      self.launcher_publisher.publish(launcher_msg)

    elif self.LT == 1:
      self.LT = 0
      arm_msg = Float64()
      arm_msg.data = 0.0
      self.arm_publisher.publish(arm_msg)

      solenoid_msg = UInt8()
      solenoid_msg.data = 0x00
      self.solenoid_publisher.publish(solenoid_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('state', anonymous=True)
        joy = joy()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass