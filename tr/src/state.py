#!/usr/bin/env python
### coding: UTF-8
import math
import rospy
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JoyFeedbackArray
from sensor_msgs.msg import JoyFeedback
from std_msgs.msg import UInt8
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

import actionlib
import tf
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def goal_pose(pose):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'omni4/map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]

    return goal_pose

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

    self.counter = 0
    self.last_status = [0]*15
    self.next = 0
    self.kick = 0 
    self.vel_mode = 0
    self.back = 0
    self.progress = 0

    self.mode_changed = 1

    self.joy_subscriber = rospy.Subscriber('joy',Joy, self.joyCallback)
    self.joy_feedback_publisher = rospy.Publisher('set_feedback',JoyFeedbackArray,queue_size=1000)
    self.slide_publisher = rospy.Publisher('slide',Float64, queue_size=1000)
    self.vel_publisher = rospy.Publisher('cmd_vel',Twist,queue_size=1)
    self.kick_publisher = rospy.Publisher('kick',Float64,queue_size=1000)
    self.cmd_publisher = rospy.Publisher('cmd',UInt8,queue_size=1000)
    self.status_publisher = rospy.Publisher('status',UInt8,queue_size=1000)
    self.result_subscriber = rospy.Subscriber('result',UInt8,self.ResultCallback)
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

    if self.last_status[self.ButtonStart] == 0 and data.buttons[self.ButtonStart] == 1:
      self.vel_mode ^= 1
      self.mode_changed = 1

    elif self.last_status[self.ButtonRB] == 0 and data.buttons[self.ButtonRB] == 1:
      self.disable = 1

    elif self.last_status[self.ButtonLB] == 0 and data.buttons[self.ButtonLB] == 1:
      self.enable = 1

    elif self.last_status[self.ButtonA] == 0 and data.buttons[self.ButtonA]==1:
      self.next=1

    elif self.last_status[self.ButtonB] == 0 and data.buttons[self.ButtonB]==1:
      self.kick ^= 1

    elif self.last_status[self.ButtonSelect] == 0 and data.buttons[self.ButtonSelect]==1:
      self.back = 1
    
    self.save_last_stat(data)

  def timerCallback(self,event):
    if self.vel_mode and self.mode_changed :
      joy_feedback = JoyFeedbackArray()
      feedback = JoyFeedback()
      feedback.type = 0 #LED
      feedback.id = 1 #green
      feedback.intensity = 60
      joy_feedback.array.append(feedback)
      self.joy_feedback_publisher.publish(joy_feedback)

      status_msg = UInt8()
      status_msg.data = self.progress + 1
      self.status_publisher.publish(status_msg)

      self.mode_changed = 0

    elif self.vel_mode == 0:

      if self.mode_changed:
        joy_feedback = JoyFeedbackArray()
        feedback = JoyFeedback()
        feedback.type = 0 #LED
        feedback.id = 2 #blue
        feedback.intensity = 60
        joy_feedback.array.append(feedback)
        self.joy_feedback_publisher.publish(joy_feedback)

        status_msg = UInt8()
        status_msg.data = 0
        self.status_publisher.publish(status_msg)
        self.mode_changed = 0

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
    
    if self.next==1:
      if self.counter == 0:
        slide_msg = Float64()
        slide_msg.data = -0.52
        self.slide_publisher.publish(slide_msg)
        self.counter += 1
        rospy.loginfo("0")
      else:
        slide_msg = Float64()
        slide_msg.data = 0
        self.slide_publisher.publish(slide_msg)
        self.counter = 0
        rospy.loginfo("slide")
      self.next=0

    if self.back == 1:
      client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
      client.wait_for_server()
      pose = [(1.0,1.0,0.0),(0.0,0.0,0.0,-1.0)]
      goal = goal_pose(pose)
      client.send_goal(goal)
      # 結果が返ってくるまで30.0[s] 待つ.ここでブロックされる.          
      succeeded = client.wait_for_result(rospy.Duration(30));
      # 結果を見て,成功ならSucceeded,失敗ならFailedと表示               
      state = client.get_state();
      if succeeded:
        rospy.loginfo("Succeeded:"+str(state))
      else:
        rospy.loginfo("Failed:"+str(state))
      self.back = 0
      self.vel_mode = 0

    if self.kick == 1:
      kick_msg = Float64()
      kick_msg.data = -50
      self.kick_publisher.publish(kick_msg)

    else:
      kick_msg = Float64()
      kick_msg.data = 1
      self.kick_publisher.publish(kick_msg)

  def ResultCallback(self,data):
    if data.data == 1:
      rospy.logwarn("Succeeded:")
      self.vel_mode = 0
      self.mode_changed = 1
      self.progress += 1
       

if __name__ == '__main__':
    try:
        rospy.init_node('state', anonymous=True)
        rospy.logwarn("Started")
        joy = joy()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


# def goal_pose(pose):
#     goal_pose = MoveBaseGoal()
#     goal_pose.target_pose.header.frame_id = 'map'
#     goal_pose.target_pose.pose.position.x = pose[0][0]
#     goal_pose.target_pose.pose.position.y = pose[0][1]
#     goal_pose.target_pose.pose.position.z = pose[0][2]
#     goal_pose.target_pose.pose.orientation.x = pose[1][0]
#     goal_pose.target_pose.pose.orientation.y = pose[1][1]
#     goal_pose.target_pose.pose.orientation.z = pose[1][2]
#     goal_pose.target_pose.pose.orientation.w = pose[1][3]

#     return goal_pose


# if __name__ == '__main__':
#     rospy.init_node('patrol')
#     listener = tf.TransformListener()

#     client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
#     client.wait_for_server()
#     listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(4.0))

#     pose = [(-24.1430511475,-1.39947879314,0.0),(0.0,0.0,0.712479235583,0.701693194255)]
#     goal = goal_pose(pose)
#     client.send_goal(goal)