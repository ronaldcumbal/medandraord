#!/usr/bin/env python
# license removed for brevity
import sys
import rospy

from azure_services.speech_recognition import SpeechRecognizer

if __name__ == '__main__':
    node_name = "microphone_left"
    rospy.init_node(node_name, anonymous=False, log_level=rospy.DEBUG)
    language = rospy.get_param("language")
    device_name = rospy.get_param("device_name_left")
    #device_name = "Jabra UC VOICE 150a MS: USB Audio (hw:1,0)" 
    # device_name = "Jabra UC VOICE 150a MS mono: USB Audio (hw:1,0)"

    sys_path = "/home/ronald/github"
    try:
        recog = SpeechRecognizer(sys_path, device_name, node_name, language=language)
    except SpeechRecognizer.InvalidDevice as e:
        rospy.logerr(e.message)
        sys.exit(1)
