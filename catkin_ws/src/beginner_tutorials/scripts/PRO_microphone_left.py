#!/usr/bin/env python3.8
# license removed for brevity

import os
import sys
import rospy

from speech_detection.speech_activation import SpeechActivation

if __name__ == '__main__':
    node_name = "microphone_left"
    rospy.init_node(node_name, anonymous=False, log_level=rospy.DEBUG)
    language = rospy.get_param("language")
    device_name = rospy.get_param("device_name_left")

    sys_path = "/home/ronald/github"
    try:
        SpeechActivation(sys_path, device_name, node_name, INITIAL_SPEECH_THRESHOLD=0.014)
    except Exception as e:
        rospy.logerr(e)
        sys.exit(1)
