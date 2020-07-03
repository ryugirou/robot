#!/usr/bin/env python
# coding: UTF-8
import rospy
import smach
import smach_ros
import joy_handler
import action_handler
from trajectory import Trajectorys
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
        if index >= len(list):
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
    if robot_name == "tr":
      rospy.loginfo("tr")
      actions = action_handler.ActionsTr()
      # list = \
      # [
      # Trajectorys.TRSZ_TO_RZ,

      # Trajectorys.RZ_TO_TS1,
      # Trajectorys.TS1_TO_RZ2,

      # Trajectorys.RZ_TO_TS2,
      # Trajectorys.TS2_TO_RZ3,

      # Trajectorys.RZ_TO_TS3,
      # Trajectorys.TS3_TO_WP,

      # Trajectorys.WP_TO_KZ,
      # Trajectorys.KZ_TO_RZ,

      # Trajectorys.RZ_TO_KZ,
      # Trajectorys.KZ_TO_RZ,

      # Trajectorys.RZ_TO_KZ,
      # Trajectorys.KZ_TO_RZ3,

      # Trajectorys.RZ_TO_TS4,
      # Trajectorys.TS4_TO_WP,

      # Trajectorys.WP_TO_KZ,
      # Trajectorys.KZ_TO_RZ3,

      # Trajectorys.RZ_TO_TS5,
      # Trajectorys.TS5_TO_WP,

      # Trajectorys.WP_TO_KZ,
      # Trajectorys.KZ_TO_RZ
      # ]
      list = \
      [
      Trajectorys.TRSZ_TO_RZ,

      Trajectorys.RZ_TO_TS1,
      Trajectorys.TS1_TO_RZ2,

      Trajectorys.RZ_TO_TS2,
      Trajectorys.TS2_TO_RZ3,

      Trajectorys.RZ_TO_TS3,
      Trajectorys.TS3_TO_RZ3,
      
      Trajectorys.RZ3_TO_TS4,
      Trajectorys.TS4_TO_KZ,

      Trajectorys.KZ_TO_RZ,
      Trajectorys.RZ_TO_KZ,

      Trajectorys.KZ_TO_KZ2,
      Trajectorys.KZ_TO_KZ3,
      
      Trajectorys.KZ_TO_RZ3,

      Trajectorys.RZ3_TO_TS5,
      Trajectorys.TS5_TO_KZ
      ]
    elif robot_name == "pr":
      rospy.loginfo("pr")
      actions = action_handler.ActionsPr()
      list = [
        Trajectorys.RZ1,
        Trajectorys.PRSZ,

        Trajectorys.RZ2,
        Trajectorys.PRSZ,

        Trajectorys.RZ3,
        Trajectorys.PRSZ,

        Trajectorys.RZ3,
        Trajectorys.PRSZ,

        Trajectorys.RZ3,
        Trajectorys.PRSZ,
      ]*100 
    else:
      rospy.loginfo(robot_name)
      actions = action_handler.ActionsVirtual()
      list = [
        Trajectorys.TRSZ_TO_RZ,

        Trajectorys.RZ_TO_TS1,
        Trajectorys.TS1_TO_RZ2,

        Trajectorys.RZ_TO_TS2,
        Trajectorys.TS2_TO_RZ3,

        Trajectorys.RZ_TO_TS3,
        Trajectorys.TS3_TO_RZ3,
        
        Trajectorys.RZ3_TO_TS4,
        Trajectorys.TS4_TO_KZ,

        Trajectorys.KZ_TO_RZ,
        Trajectorys.RZ_TO_KZ,

        Trajectorys.KZ_TO_KZ2,
        Trajectorys.KZ_TO_KZ3,
        
        Trajectorys.KZ_TO_RZ3,

        Trajectorys.RZ3_TO_TS5,
        Trajectorys.TS5_TO_KZ
      ]

    # Create a SMACH state machine
    sm_top = smach.StateMachine(outcomes=['Finished'])

    # Open the container
    with sm_top:
        smach.StateMachine.add('Init', Init(), transitions={'initialized': 'Manual'})
        smach.StateMachine.add('Manual', Manual(), transitions={'->Auto': 'Auto','->Finished':'Finished'})
        smach.StateMachine.add('Auto', Auto(), transitions={'->Manual': 'Manual','->Finished':'Finished'})
        # smach.StateMachine.add('Kick', Kick(), transitions={'kicked': 'Manual'})


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
