#!/usr/bin/env python
### coding: UTF-8

import rospy
import smach
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# define state Move
class Move(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['goal_reached','aborted'])
    self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    self.goal = MoveBaseGoal()
    self.goal.target_pose.header.frame_id = "map"

  def execute(self, userdata):
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 0.5
    goal.target_pose.pose.orientation.w = 1.0
    # サーバーにgoalを送信                                              
    self.ac.send_goal(self.goal);
    # 結果が返ってくるまで30.0[s] 待つ。ここでブロックされる。          
    succeeded = self.ac.wait_for_result(rospy.Duration(30));
    # 結果を見て、成功ならSucceeded、失敗ならFailedと表示               
    state = self.ac.get_state();
    if succeeded:
      rospy.loginfo("Succeeded:"+str(state))
      return 'goal_reached'
    else:
      rospy.loginfo("Failed:"+str(state))
      return 'aborted'

# define state Recover
class Recover(smach.State):
    def __init__(self):
      smach.State.__init__(self, outcomes=['recovered'])
    def execute(self, userdata):
      rospy.loginfo("Recovering")
      return 'recovered'

# define state Receive
class Receive(smach.State):
    def __init__(self):
      smach.State.__init__(self, outcomes=['received'])
    def execute(self, userdata):
      rospy.loginfo("Received")
      return 'received'

# define state Place
class Place(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
    def execute(self, userdata):

# define state Kick
class Kick(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
    def execute(self, userdata):

# define state Init
class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['initialized'])
    def execute(self, userdata):

# main
def main():
    rospy.init_node('state_machine')

    # Create a SMACH state machine
    sm_top = smach.StateMachine(outcomes=['finished'])

    # Open the container
    with sm_top:
        # Add states to the container
        smach.StateMachine.add('Init', Init(), transitions={'initialized':'CATCH'})

        # Create the sub SMACH state machine
        sm_catch = smach.StateMachine(outcomes=['catched'])
        # Open the container
        with sm_catch:
          # Add states to the container
          smach.StateMachine.add('Move', Move(), transitions={'goal_reached':'Receive','aborted':'Recover'})
          smach.StateMachine.add('Recover', Recover(), transitions={'recovered':'Move'})
          smach.StateMachine.add('Receive', Receive(), transitions={'received':'catched'})
        smach.StateMachine.add('CATCH', sm_catch,transitions={'catched':'TRY'})
        
        # Create the sub SMACH state machine
        sm_try = smach.StateMachine(outcomes=['nextball','end_sequence'])
        # Open the container
        with sm_try:
          # Add states to the container
          smach.StateMachine.add('Move', Move(), transitions={'goal_reached':'Receive','aborted':'Recover'})
          smach.StateMachine.add('Recover', Recover(), transitions={'recovered':'Move'})
          smach.StateMachine.add('Place', Place(), transitions={'placed':'CATCH'})
        smach.StateMachine.add('TRY', sm_try,transitions={'nextball':'CATCH','end_sequence':'KICK'})
        

        # Create the sub SMACH state machine
        sm_goalkick = smach.StateMachine(outcomes=['kicked','end_sequence'])
        # Open the container
        with sm_goalkick:
          # Add states to the container
          smach.StateMachine.add('Move', Move(), transitions={'goal_reached':'Receive','aborted':'Recover'})
          smach.StateMachine.add('Recover', Recover(), transitions={'recovered':'Move'})
          smach.StateMachine.add('Kick', Kick(), transitions={'initialized':'CATCH'})
        smach.StateMachine.add('GOALKICK', sm_goalkick,transitions={'kicked':'CATCH','end_sequence':'finished'})

    # Execute SMACH plan
    outcome = sm_top.execute()


if __name__ == '__main__':
    main()
