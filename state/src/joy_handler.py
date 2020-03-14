#!/usr/bin/env python
# coding: UTF-8

import rospy
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JoyFeedbackArray
from sensor_msgs.msg import JoyFeedback
from enum import IntEnum

class Joy_Handler:
    def __init__(self):
        self.ButtonNames = rospy.get_param("~joy")

        self.axes = [0]*6
        self.buttons = [0]*18
        self.__last_status = [0]*18

        self.joy_feedback_publisher = rospy.Publisher('set_feedback',JoyFeedbackArray,queue_size=10,latch=True)
        self.joy_subscriber = rospy.Subscriber('joy',Joy, self.__joyCallback)
        self.LEDColor = IntEnum('LEDColor','RED GREEN BLUE')

    def __joyCallback(self, data):
        for i,button in enumerate(data.buttons,0):
            if(self.__last_status[i] != button):
                self.buttons[i] = 1 if self.__last_status[i] == 1 else 0
                self.__last_status[i] = button
                #rospy.loginfo("status_changed")

        for i,axe in enumerate(data.axes,0):
            self.axes[i] = axe

    def ClearButtons(self):
        self.buttons = [0]*18

    def GetButtonState(self,data):
        if self.buttons[data] == 1:
            self.buttons[data] = 0
            return True
        else:
            return False

    def Set_LEDColor(self, LEDColor):
        joy_feedback = JoyFeedbackArray()
        feedback = JoyFeedback()
        feedback.type = 0
        feedback.id = LEDColor - 1
        feedback.intensity = 60
        joy_feedback.array.append(feedback)
        self.joy_feedback_publisher.publish(joy_feedback)

    def Rumble(self):
        joy_feedback = JoyFeedbackArray()
        feedback = JoyFeedback()
        feedback.type = 1
        feedback.id = 0
        feedback.intensity = 0.8
        joy_feedback.array.append(feedback)
        self.joy_feedback_publisher.publish(joy_feedback)