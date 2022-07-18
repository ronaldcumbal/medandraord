#!/usr/bin/env python
# license removed for brevity
import rospy
import random
import time
from beginner_tutorials.msg import Transcript, ActiveSpeech

def talker():
    node_name = "microphone_fake"
    rospy.init_node(node_name, anonymous=False, log_level=rospy.DEBUG)
    pub_transc = rospy.Publisher('transcript_left', Transcript, queue_size=10)
    pub_speech = rospy.Publisher('speech_left', ActiveSpeech, queue_size=10)

    rate = rospy.Rate(10) # 10hz - loop 10 times per second
    while not rospy.is_shutdown():

        list_fake_messages = ["This is a white animal",
                            "An animal that lives in the north pole",
                            "It has a fluffy hair",
                            "It's a big animal that lives in the snow",
                            "It is an animal with white fur",
                            "It's an animal that hibernates",
                            "Coca Cola uses this animal for their advertisement",
                            "These are big animals that live in the cold temperatures",
                            "These animal are in dangered of climate change",
                            "Do you understand me?"]

        option = random.choice(list_fake_messages)
        msg = ActiveSpeech()
        msg.text = option.split()[0]
        msg.duration = 1.0
        msg.offset = 0.0
        msg.timestamp = time.time()
        pub_speech.publish(msg)
        rospy.logdebug("{} topic: {}".format(rospy.get_caller_id(), "speech_left"))
        rate.sleep()
        time.sleep(5.0)

        msg = Transcript()
        msg.text = option
        msg.duration = 1.0
        msg.offset = 0.0
        msg.timestamp = time.time()
        pub_transc.publish(msg)
        rospy.logdebug("{} topic: {} ".format(rospy.get_caller_id(), 'transcript_left'))
        time.sleep(5.0)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
