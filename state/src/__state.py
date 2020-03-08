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
    SZ_TO_RZ = 1
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


# define state Manual
class Manual(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['->Auto'])

    def execute(self, userdata):
        rospy.loginfo("Manual")
        joy.Set_LEDColor(joy.LEDColor.BLUE)
        actions.send_goal(0)
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            actions.teleop(joy.axes[0],joy.axes[1],joy.axes[2])
            if sum(joy.buttons) <= 0:
              continue
            elif joy.GetButtonState(4):
              actions.enable()
            elif joy.GetButtonState(6):
              actions.disable()
            elif joy.GetButtonState(10):
              return '->Auto'

# define state Manual
class Auto(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['->Manual'])

    def execute(self, userdata):
        rospy.loginfo("Auto")
        joy.Set_LEDColor(joy.LEDColor.GREEN)
        global index
        actions.send_goal(list[index])
        if index < len(list) - 1:
          index += 1
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            if actions.getResult():
              return '->Manual'
            elif joy.GetButtonState(10):
              return '->Manual'

def main():
    rospy.init_node('state', anonymous=True)
    rospy.logwarn("Started")

    global joy,actions,list
    joy = joy_handler.Joy_Handler()
    actions = action_handler.Actions()
    list = \
    [\
    Trajectorys.SZ_TO_RZ,\
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

    # list = [100] #test

    # Create a SMACH state machine
    sm_top = smach.StateMachine(outcomes=['finished'])

    # Open the container
    with sm_top:
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
