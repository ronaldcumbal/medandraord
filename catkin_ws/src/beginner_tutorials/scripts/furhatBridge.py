#!/usr/bin/env python3.8
# license removed for brevity

import os
import sys
import time
import random
import numpy as np

import rospy
import rospkg
from std_msgs.msg import String
from beginner_tutorials.msg import Transcript, GameState, RobotSpeech

from furhat_remote_api import FurhatRemoteAPI

class Furhat():
    def __init__(self):
        self.language = rospy.get_param("language")
        self.robot_present = rospy.get_param("robot_present")
        if self.robot_present:
            self.furhat = FurhatRemoteAPI(host="192.168.1.3")
            self.furhat.set_voice(name='Elinfurhat22k_HQ')

        self.backchannels = {"soft": ["#m-hm_keep_going_B#", "#m-hm_AB#","#m-hm_AC#","#m-hm_AD#","#m-hm_I'm_listening_C#"],
                             "strong": ["#m-hm_AE#","#m-hm_I'm_listening_A#"],

                            }
        self.filler_list = ["#think_A#", "#think_B#", "#think_C#"]

        self.before_speech = {"soft": ["#hes_A#","#hes_B#","#hes_C#","#hes_D#","#hes_E#",
                                                    "#hes_F#","#hes_G#","#hes_H#","#hes_I#","#hes_J#"],
                              "strong": ["#oh_A#", "#oh_B#", "#oh_C#","#oh_D#","#oh_E#"]
                            }

        self.gestures_list = ['GazeAway', 'Nod', 'Roll']
        self.attend_locations = { 'left': "-0.7,0.14,2.0",
                                  'left_averse': "-0.96,0.59,2.0",
                                  'right': "0.7,-0.14,2.0",
                                  'right_averse': "0.96,0.59,2.0",
                                  'away': "0.1,0.1,2.0",
                                  'tablet': "0.0,-0.2,2.0",
                                  'pose': "0.0,0.0,0.0",
                                  'all': "0.0,0.2,2.0"
                                  }
        # Subscribing nodes
        rospy.Subscriber('game_state', GameState, self.game_state_callback)
        rospy.Subscriber('robot_speak', RobotSpeech, self.robot_speak_callback)
        rospy.Subscriber('robot_gesture', String, self.robot_gesture_callback)
        rospy.Subscriber('robot_attend', String, self.robot_attend_callback)
        rospy.loginfo("{} initialized".format(rospy.get_caller_id()))
        rospy.spin()

    def game_state_callback(self, data):
        if data.state == "start_game":
            pass
        if data.state == "new_turn":
            pass

    def robot_speak_callback(self, data):
        if data.type_speech == "backchannel":
            type_backchannel = data.data
            text = random.choice(self.backchannels[type_backchannel])
        elif data.type_speech == "filler":
            text = random.choice(self.filler_list)
        elif data.type_speech == "guess":
            type_guess = data.data
            word = data.text
            text = "{} \pau=250\  {}?".format(random.choice(self.before_speech[type_guess]), word)
        else:
            text = data.text

        if self.robot_present:
            self.furhat.say(text=text)

    def robot_gesture_callback(self, data):
        if self.robot_present:
            self.furhat.gesture(name=data.data)

    def robot_attend_callback(self, data):
        if self.robot_present:
            self.furhat.attend(location=self.attend_locations[data.data])


if __name__ == '__main__':
    rospy.init_node('furhatBridge', anonymous=False)
    try:
        Furhat()
    except Exception as e:
        rospy.logerr(e)
        sys.exit(1)
