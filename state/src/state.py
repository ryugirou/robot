#!/usr/bin/env python
# coding: UTF-8
import rospy
import smach
import smach_ros
import joy_handler
import action_handler
from enum import IntEnum

index = 0

class Trajectorys(IntEnum):
    TEST = 0

    #TR
    TRSZ_TO_RZ = 1
    RZ_TO_TS1 = 2
    TS1_TO_RZ = 3
    RZ_TO_TS2 = 4
    TS2_TO_RZ = 5
    RZ_TO_TS3 = 6
    TS3_TO_RZ = 7
    RZ_TO_TS4 = 8
    TS4_TO_RZ = 9
    RZ_TO_TS5 = 10
    TS5_TO_RZ = 11

    TS1_TO_KZ = 12
    TS2_TO_KZ = 13
    TS3_TO_KZ = 14
    TS4_TO_KZ = 15
    TS5_TO_KZ = 16

    KZ_TO_RZ = 17
    #PR
    # PRSZ_TO_PP1 = 12
    # PP1_TO_RZ = 13
    # RZ_TO_PP2 = 14
    # PP2_TO_RZ = 15
    # RZ_TO_PP3 = 16
    # PP3_TO_RZ = 17
    # RZ_TO_PP4 = 18
    # PP4_TO_RZ = 19
    # RZ_TO_PP5 = 20
    # PP5_TO_RZ = 21


# define state Manual
class Manual(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['->Auto'])

    def execute(self, userdata):
        rospy.loginfo("Manual")
        actions.send_goal(100)
        joy.Rumble()
        joy.Set_LEDColor(joy.LEDColor.BLUE)
        while not rospy.is_shutdown():
            rospy.sleep(0.03)
            actions.teleop(joy.axes[joy.ButtonNames['AxisLeftThumbY']],joy.axes[joy.ButtonNames['AxisLeftThumbX']],joy.axes[joy.ButtonNames['AxisLeftTrigger']],joy.axes[joy.ButtonNames['AxisRightTrigger']])
            if sum(joy.buttons) <= 0:
              continue
            elif joy.GetButtonState(joy.ButtonNames['ButtonStart']):
              return '->Auto'
            else :
              for button_name,button_index in joy.ButtonNames.items():
                if not button_name in actions.ButtonNames:
                  continue
                if joy.GetButtonState(button_index):
                  actions.do(actions.ButtonNames[button_name])

# define state Init
class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['initialized'])
    def execute(self, userdata):
        if not (rospy.has_param('amcl/initial_pose_x') and rospy.has_param('amcl/initial_pose_y') and rospy.has_param('amcl/initial_pose_a')):
            actions.pose_intialize()
            rospy.loginfo("pose initialized ")
        else :
            rospy.loginfo("using last pose")
        return 'initialized'
# define state Manual
class Auto(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['->Manual'])

    def execute(self, userdata):
        rospy.loginfo("Auto")
        joy.Set_LEDColor(joy.LEDColor.GREEN)
        global index
        if index >= len(list):
          rospy.logwarn("next trajectory does not exist")
          return '->Manual'
        actions.send_goal(list[index])
        index += 1
        while not rospy.is_shutdown():
            rospy.sleep(0.03)
            if actions.getResult():
              return '->Manual'
            elif joy.GetButtonState(joy.ButtonNames['ButtonStart']):
              return '->Manual'
            else :
              for button_name,button_index in joy.ButtonNames.items():
                if not button_name in actions.ButtonNames:
                  continue
                if joy.GetButtonState(button_index):
                  actions.do(actions.ButtonNames[button_name])

def main():
    rospy.init_node('state', anonymous=True)
    rospy.logwarn("Started")

    global joy,actions,list
    joy = joy_handler.Joy_Handler()

    robot_name = rospy.get_param("~robot_name")
    if robot_name == "tr":
      rospy.loginfo("tr")
      actions = action_handler.ActionsTr()
      list = \
      [\
      Trajectorys.TRSZ_TO_RZ,\
      Trajectorys.RZ_TO_TS1,\
      Trajectorys.TS1_TO_RZ,\
      Trajectorys.RZ_TO_TS2,\
      Trajectorys.TS2_TO_RZ,\
      Trajectorys.RZ_TO_TS3,\
      Trajectorys.TS3_TO_RZ,\
      Trajectorys.RZ_TO_TS4,\
      Trajectorys.TS4_TO_RZ,\
      Trajectorys.RZ_TO_TS5,\
      Trajectorys.TS5_TO_RZ\
      ]
    elif robot_name == "pr":
      rospy.loginfo("pr")
      actions = action_handler.ActionsPr()
      list = [Trajectorys.TEST]*100 #test
    else:
      rospy.logwarn(robot_name + "is not a valid name")

    # Create a SMACH state machine
    sm_top = smach.StateMachine(outcomes=['finished'])

    # Open the container
    with sm_top:
        smach.StateMachine.add('Init', Init(), transitions={'initialized': 'Manual'})
        smach.StateMachine.add('Manual', Manual(), transitions={'->Auto': 'Auto'})
        smach.StateMachine.add('Auto', Auto(), transitions={'->Manual': 'Manual'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/PR')
    sis.start()

    # Execute SMACH plan
    outcome = sm_top.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
