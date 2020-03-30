#!/usr/bin/env python
### coding: UTF-8
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseWithCovarianceStamped , Quaternion
from tf.transformations import quaternion_from_euler
import threading
import tf2_ros

def fire_and_forget(f):
  def wrapped(self):
    threading.Thread(target=f,args=[self]).start()
  return wrapped

class Actions:
  def __init__(self):
    self.ButtonNames = rospy.get_param("~tasks")
    #tf
    self.__tfBuffer = tf2_ros.Buffer()
    self.__listener = tf2_ros.TransformListener(self.__tfBuffer)

    self.__cmd_publisher = rospy.Publisher('cmd',UInt8,queue_size=10,latch=True)
    self.__vel_publisher = rospy.Publisher('cmd_vel',Twist,queue_size=1)
    self.__goal_publisher = rospy.Publisher('goal',UInt8,queue_size=10)
    #PR
    if rospy.get_param("~robot_name") == "pr":
      self.__pass_publisher = rospy.Publisher('pass',Float64,queue_size=10)
      self.__arm_publisher = rospy.Publisher('arm',Float64,queue_size=10)
      self.__solenoid_publisher = rospy.Publisher('solenoid',UInt8,queue_size=10)
      self.__slide_publisher = rospy.Publisher('slide',Float64,queue_size=10)
      self.__limit_switch_sub = rospy.Subscriber('limit_switch',UInt8,lambda msg: self.Pick() if msg.data else 0)
      self.__picked = True
    #TR
    elif rospy.get_param("~robot_name") == "tr":
      self.__kick_publisher = rospy.Publisher('kick',Float64,queue_size=10)

    self.__initialpose_publisher = rospy.Publisher('initialpose',PoseWithCovarianceStamped,queue_size=10,latch=True)
    self.__result = 0
    self.__max_lin = rospy.get_param("~max_lin")
    self.__max_ang = rospy.get_param("~max_ang")
    self.__result_subscriber = rospy.Subscriber('result',UInt8, self.__resultCallback)
    

  def pose_intialize(self):
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.frame_id = rospy.get_param('amcl/global_frame_id','default_value')
    initial_pose.header.stamp = rospy.Time.now()
    initial_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
    initial_pose.pose.pose.position.x = rospy.get_param('~initial_pose_x')
    initial_pose.pose.pose.position.y = rospy.get_param('~initial_pose_y')
    initial_pose.pose.pose.position.z = 0
    initial_pose.pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, rospy.get_param('~initial_pose_a')))
    self.__initialpose_publisher.publish(initial_pose)
    
  def Homing(self):
    cmd_msg = UInt8()
    cmd_msg.data = 0x10
    self.__cmd_publisher.publish(cmd_msg)
    rospy.loginfo("homing")

  def Enable(self):
    cmd_msg = UInt8()
    cmd_msg.data = 1
    self.__cmd_publisher.publish(cmd_msg)
    rospy.loginfo("enable")

  def Disable(self):
    cmd_msg = UInt8()
    cmd_msg.data = 0
    self.__cmd_publisher.publish(cmd_msg)
    rospy.loginfo("disable")

  def teleop(self,vel_x,vel_y,vel_z_l,vel_z_r):
      norm = math.sqrt(vel_x**2+vel_y**2)
      if norm > 1.0:
        vel_x = vel_x / norm
        vel_y = vel_y / norm
      else:
        vel_x = vel_x
        vel_y = vel_y

      vel_x *= self.__max_lin
      vel_y *= self.__max_lin

      vel_z = vel_z_l - vel_z_r
      vel_z *= self.__max_ang

      vel_msg = Twist()
      vel_msg.linear.x = vel_x
      vel_msg.linear.y = vel_y
      vel_msg.angular.z = vel_z
      self.__vel_publisher.publish(vel_msg)

  def send_goal(self,trajectory):
    goal_msg = UInt8()
    goal_msg.data = trajectory
    self.__goal_publisher.publish(goal_msg)

  def __resultCallback(self,data):
    self.__result = data.data

  def getResult(self):
    if self.__result:
      self.__result = 0
      return True
    else:
      return False

  #PR
  @fire_and_forget
  def Arm(self):
    arm_msg = Float64()
    arm_msg.data = -3.65
    self.__arm_publisher.publish(arm_msg)
    self.__picked = False
    while(not self.__picked):
      rospy.sleep(0.5)
    arm_msg.data = 0.0
    self.__arm_publisher.publish(arm_msg)

  @fire_and_forget
  def Pick(self):
    if self.__picked: return
    self.__picked = True
    pick_msg = UInt8()
    pick_msg.data = 0xFF
    self.__solenoid_publisher.publish(pick_msg)
    rospy.sleep(1)
    pick_msg.data = 0x00
    self.__solenoid_publisher.publish(pick_msg)
    rospy.sleep(1)

  @fire_and_forget
  def Slide(self):
    self.Pass()
    rospy.sleep(1)
    slide_msg = Float64()
    slide_msg.data = -0.6
    self.__slide_publisher.publish(slide_msg)
    rospy.sleep(5)
    slide_msg.data = 0
    self.__slide_publisher.publish(slide_msg)

  @fire_and_forget
  def Pass(self):
    pass_msg = Float64()
    pass_msg.data = 140
    self.__pass_publisher.publish(pass_msg)
    rospy.sleep(4)
    pass_msg.data = 0
    self.__pass_publisher.publish(pass_msg)

  #TR
  @fire_and_forget
  def Kick(self):
    kick_msg = Float64()
    # kick_msg.data = -23.7
    kick_msg.data = -26
    self.__kick_publisher.publish(kick_msg)
    rospy.sleep(2)
    kick_msg.data = 0
    self.__kick_publisher.publish(kick_msg)


  @fire_and_forget
  def Try(self):
    try_msg = UInt8()
    try_msg.data = 0xFF
    self.__solenoid_publisher.publish(try_msg)
    rospy.sleep(3)
    try_msg = 0x00
    self.__solenoid_publisher.publish(try_msg)

  def Hold(self):
    hold_msg = UInt8()


  def do(self,string):
    eval("self."+ string)()
    # self.doSomeWork()