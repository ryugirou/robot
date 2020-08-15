#!/usr/bin/env python
# coding: UTF-8
import rospy
import smach
import smach_ros
import joy_handler
import action_handler2
import time

index = 0

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
class Manual(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['->Auto','->Finished'])

    def execute(self, userdata):
        rospy.loginfo("Manual")
        actions.send_goal("manual")
        joy.Rumble()
        joy.Set_LEDColor(joy.LEDColor.BLUE)
        while not rospy.is_shutdown():
            rospy.sleep(0.03)
            actions.teleop(joy.axes[joy.ButtonNames['AxisLeftThumbY']],joy.axes[joy.ButtonNames['AxisLeftThumbX']],joy.axes[joy.ButtonNames['AxisLeftTrigger']],joy.axes[joy.ButtonNames['AxisRightTrigger']])
            if sum(joy.buttons) <= 0:
              continue
            elif joy.GetButtonState(joy.ButtonNames['ButtonStart']):
              return '->Auto'
            elif joy.GetButtonState(joy.ButtonNames['ButtonTouchpad']):
              global index
              index = 0
              rospy.loginfo("reset")
              for __ in range(3) :
                joy.Set_LEDColor(joy.LEDColor.RED)
                rospy.sleep(1)
                joy.Set_LEDColor(joy.LEDColor.BLUE)
                rospy.sleep(1)
              continue
            else :
              for button_name,button_index in joy.ButtonNames.items():
                if not button_name in actions.ButtonNames:
                  continue
                if joy.GetButtonState(button_index):
                  actions.do(actions.ButtonNames[button_name])
        return '->Finished'

# define state Manual
class Auto(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['->Manual','->Finished'])

    def execute(self, userdata):
        rospy.loginfo("Auto")
        joy.Set_LEDColor(joy.LEDColor.GREEN)
        global index
        if index > len(list):
          rospy.logwarn("next trajectory does not exist")
          return '->Manual'
        actions.send_goal(list[index])
        start = time.time()
        while not rospy.is_shutdown():
            rospy.sleep(0.03)
            if actions.getResult():
              elapsed_time = time.time() - start
              rospy.loginfo("%s : %f sec",list[index],elapsed_time)
              index += 1
              return '->Manual'
            elif joy.GetButtonState(joy.ButtonNames['ButtonStart']):
              index += 1
              return '->Manual'
            elif joy.GetButtonState(joy.ButtonNames['ButtonTouchpad']):
              global index
              index = 0
              rospy.loginfo("reset")
              for __ in range(3) :
                joy.Set_LEDColor(joy.LEDColor.RED)
                rospy.sleep(1)
                joy.Set_LEDColor(joy.LEDColor.BLUE)
                rospy.sleep(1)
              continue
            else :
              for button_name,button_index in joy.ButtonNames.items():
                if not button_name in actions.ButtonNames:
                  continue
                if joy.GetButtonState(button_index):
                  actions.do(actions.ButtonNames[button_name])
        return '->Finished'

def main():
    rospy.init_node('state', anonymous=True)
    rospy.logwarn("Started")

    global joy,actions,list
    joy = joy_handler.Joy_Handler()

    robot_name = rospy.get_param("~robot_name")
    Field = "BlueField" if rospy.get_param("~color")=="blue" else "RedField"
    if robot_name == "tr":
      rospy.loginfo("tr")
      actions = action_handler2.ActionsTr()
      list = [
        "trsz_to_rz",
        "rz_to_ts1",
        "ts1_to_rz2",
        "rz2_to_ts2",
        "ts2_to_rz3",
        "rz3_to_ts3",
        "ts3_to_rz3",
        "rz3_to_ts4",
        "ts4_to_kz",
        "kz_to_rz",
        "rz_to_kz",
        "kz_to_kz2",
        "kz2_to_kz3",
        "kz3_to_rz3",
        "rz3_to_ts5",
        "ts5_to_kz"
      ]
    elif robot_name == "pr":
      rospy.loginfo("pr")
      actions = action_handler2.ActionsPr()
      list = [
        "trsz_to_rz",
        "rz_to_ts1",
        "ts1_to_rz2",
        "rz2_to_ts2",
        "ts2_to_rz3",
        "rz3_to_ts3",
        "ts3_to_rz3",
        "rz3_to_ts4",
        "ts4_to_kz",
        "kz_to_rz",
        "rz_to_kz",
        "kz_to_kz2",
        "kz2_to_kz3",
        "kz3_to_rz3",
        "rz3_to_ts5",
        "ts5_to_kz"
      ]
      robot_name = "tr"
    else:
      rospy.loginfo(robot_name)
      actions = action_handler2.ActionsVirtual()
      list = [
        "trsz_to_rz",
        "rz_to_ts1",
        "ts1_to_rz2",
        "rz2_to_ts2",
        "ts2_to_rz3",
        "rz3_to_ts3",
        "ts3_to_rz3",
        "rz3_to_ts4",
        "ts4_to_kz",
        "kz_to_rz",
        "rz_to_kz",
        "kz_to_kz2",
        "kz2_to_kz3",
        "kz3_to_rz3",
        "rz3_to_ts5",
        "ts5_to_kz"
      ]
      robot_name = "tr"

    for index,object in enumerate(list):
      list[index] = robot_name + "/" +Field + "/" + object

    # Create a SMACH state machine
    sm_top = smach.StateMachine(outcomes=['Finished'])

    # Open the container
    with sm_top:
        smach.StateMachine.add('Init', Init(), transitions={'initialized': 'Manual'})
        smach.StateMachine.add('Manual', Manual(), transitions={'->Auto': 'Auto','->Finished':'Finished'})
        smach.StateMachine.add('Auto', Auto(), transitions={'->Manual': 'Manual','->Finished':'Finished'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/PR')
    sis.start()

    # Execute SMACH plan
    outcome = sm_top.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    try:
        main()
    except rospy.exceptions.ROSInterruptException: pass
