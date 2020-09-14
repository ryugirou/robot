#!/usr/bin/env python
### coding: UTF-8
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseWithCovarianceStamped , Quaternion
from tf.transformations import quaternion_from_euler
import threading
import tf2_ros

def fire_and_forget(f):
  def wrapped(self):
    threading.Thread(target=f,args=[self]).start()
  return wrapped

class ActionsVirtual(object) :
    def __init__(self):
      self.ButtonNames = rospy.get_param("~tasks")
      #tf
      self.__tfBuffer = tf2_ros.Buffer()
      self.__listener = tf2_ros.TransformListener(self.__tfBuffer)

      self.__cmd_publisher = rospy.Publisher('cmd',UInt8,queue_size=10,latch=True)
      self._vel_publisher = rospy.Publisher('cmd_vel',Twist,queue_size=1)
      self.__goal_publisher = rospy.Publisher('goal',String,queue_size=10)

      self.__initialpose_publisher = rospy.Publisher('initialpose',PoseWithCovarianceStamped,queue_size=10,latch=True)
      self.__result = 0
      self._max_lin = rospy.get_param("~max_lin")
      self._max_ang = rospy.get_param("~max_ang")
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

        vel_x *= self._max_lin
        vel_y *= self._max_lin

        vel_z = vel_z_l - vel_z_r
        vel_z *= self._max_ang

        vel_msg = Twist()
        vel_msg.linear.x = vel_x
        vel_msg.linear.y = vel_y
        vel_msg.angular.z = vel_z
        self._vel_publisher.publish(vel_msg)

    def send_goal(self,trajectory):
      goal_msg = String()
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

    def do(self,string):
      eval("self."+ string)()

class ActionsPr(ActionsVirtual):
    PICK_POSITION = 0b000001

    def __init__(self):
      super(ActionsPr,self).__init__()
      self.__pass_publisher = rospy.Publisher('pass',Float64,queue_size=10)
      self.__arm_publisher = rospy.Publisher('arm',Float64,queue_size=10)
      self.__slide_publisher = rospy.Publisher('slide',Float64,queue_size=10)
      self.__solenoid_publisher = rospy.Publisher('solenoid',UInt8,queue_size=10)
      self.__limit_switch_sub = rospy.Subscriber('limit_switch',UInt8,lambda msg: self.Pick() if msg.data else 0)
      self.__passed = False
      self.__picked = True
      self.__solenoid_msg = UInt8()
      self.__solenoid_msg.data = 0b000000



    @fire_and_forget
    def Arm(self):
      arm_msg = Float64()
      arm_msg.data = -3.65
      self.__arm_publisher.publish(arm_msg)
      self.__picked = False
      while(not self.__picked):
        rospy.sleep(1)
      arm_msg.data = 0.0
      self.__arm_publisher.publish(arm_msg)

    @fire_and_forget
    def Pick(self):
      if self.__picked: return
      self.__picked = True
      self.__solenoid_msg.data |= ActionsPr.PICK_POSITION
      self.__solenoid_publisher.publish(self.__solenoid_msg)
      rospy.sleep(2)
      self.__solenoid_msg.data &= ~ActionsPr.PICK_POSITION
      self.__solenoid_publisher.publish(self.__solenoid_msg)
      rospy.sleep(1)

    @fire_and_forget
    def Slide(self):
      slide_msg = Float64()
      slide_msg.data = -0.6
      self.__slide_publisher.publish(slide_msg)
      self.__passed = True
      while self.__passed:
        rospy.sleep(0.1)
      rospy.sleep(1)
      slide_msg.data = 0
      self.__slide_publisher.publish(slide_msg)

    @fire_and_forget
    def Pass(self):
      pass_msg = Float64()
      # pass_msg.data = 140
      pass_msg.data = 150
      self.__pass_publisher.publish(pass_msg)
      while not self.__passed:
        rospy.sleep(0.1)
      rospy.sleep(3)
      self.__passed = False
      pass_msg.data = 0
      self.__pass_publisher.publish(pass_msg)

class ActionsTr(ActionsVirtual):

    ARM_POSITION = 0b000001
    HOLD_POSITION = 0b000010
    SLIDE_POSITION = 0b000100

    def __init__(self):
      super(ActionsTr,self).__init__()
      self.__kick_publisher = rospy.Publisher('kick',Float64,queue_size=10)
      self.__kick_pos_publisher = rospy.Publisher('kick_pos',Float64,queue_size=10)
      self.__limit = 0
      self.__try_publisher = rospy.Publisher('try',Float64,queue_size=10)
      self.__limit_switch_sub = rospy.Subscriber('limit_switch',UInt8,self.__limitCallback)

    #over_ride
    def teleop(self,vel_x,vel_y,vel_z_l,vel_z_r):
      norm = math.sqrt(vel_x**2+vel_y**2)
      if norm > 1.0:
          vel_x = vel_x / norm
          vel_y = vel_y / norm
      else:
          vel_x = vel_x
          vel_y = vel_y

      vel_x *= self._max_lin
      vel_y *= self._max_lin

      vel_z = vel_z_l - vel_z_r
      vel_z *= self._max_ang

      vel_msg = Twist()
      vel_msg.linear.x = -vel_x + vel_y
      vel_msg.linear.y = -vel_y - vel_x
      vel_msg.angular.z = vel_z
      self._vel_publisher.publish(vel_msg)

    def __limitCallback(self,data):
      self.__limit = data.data

    @fire_and_forget
    def Kick(self):
      kick_msg = Float64()
      kick_msg.data = -22
      self.__kick_publisher.publish(kick_msg)
      rospy.sleep(1.5)
      self.Enable()
      rospy.sleep(1)
      self.SetPosition()

    @fire_and_forget
    def Try(self):
      try_msg = Float64()
      try_msg.data = -1.5
      self.__try_publisher.publish(try_msg)
      rospy.sleep(2)
      try_msg.data = 0
      self.__try_publisher.publish(try_msg)

    def SetPosition(self):
      reset_msg = Float64()
      reset_msg.data = 0
      self.__kick_pos_publisher.publish(reset_msg)


    def Tune(self):
      self.__limit = 0
      while self.__limit == 0:
        rospy.sleep(0.1)
        self.teleop(0.1,0,0,0)
      self.teleop(0,0,0,0)
