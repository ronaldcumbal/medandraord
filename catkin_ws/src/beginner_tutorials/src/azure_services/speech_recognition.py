import os
from struct import pack
from threading import Thread

import os
import time
import string
import json
import pyaudio

import azure.cognitiveservices.speech as speechsdk

import rospy
from beginner_tutorials.msg import Transcript, ActiveSpeech

class SpeechRecognizer(object):

    class InvalidDevice(ValueError):
        pass

    def __init__(self, sys_path, device_name, microphone_name, language="en-US"):
        self.sys_path = sys_path
        self.device_name = device_name
        self.language = language
        self.node_name = microphone_name
        self.topic_transc_name = 'transcript_' + self.node_name
        self.topic_speech_name = 'speech_' + self.node_name
        self.pub_transcript = rospy.Publisher(self.topic_transc_name, Transcript, queue_size=10)
        self.pub_speech = rospy.Publisher(self.topic_speech_name, ActiveSpeech, queue_size=10)
        self._init_recognizer()
        self.run()

    def _init_recognizer(self):
        device_index = None
        self.pa_handler = pyaudio.PyAudio()
        for i in range(self.pa_handler.get_device_count()):
            dev = self.pa_handler.get_device_info_by_index(i)
            print('{:2} {:10} maxInChannels: {:3} maxOutChannels: {:3}'.format(i,
                dev['name'],
                dev['maxInputChannels'],
                dev['maxOutputChannels']))
            if self.device_name == dev['name']:
                print('\nUsing {} {}'.format(i, dev['name']))
                device_index = i
                device_id = self.device_name[self.device_name.find("(")+1:self.device_name.find(")")]
                break

        try:
            rospy.loginfo("{} using device: {}".format(
                self.node_name,
                self.pa_handler.get_device_info_by_index(device_index)['name'])
            )
        except IOError:
            self.terminate()
            raise self.InvalidDevice(
                'Invalid device ID: {}. Available devices listed in rosparam '
                '/ros_speech2text/available_audio_device'.format(self.device_name))

        key_path = os.path.join(self.sys_path, "medandraord/catkin_ws/src/beginner_tutorials/src/azure_services/azure_keys.json")
        with open(key_path, "r") as infile:
             key_data = json.loads(infile.read())
        SPEECH_KEY = key_data["SPEECH_KEY"]
        SERVICE_REGION = key_data["SERVICE_REGION"]

        speech_config = speechsdk.SpeechConfig(subscription=SPEECH_KEY, region=SERVICE_REGION)
        speech_config.speech_recognition_language = self.language

        print("Using device_id: {} ".format(device_id))
        audio_config = speechsdk.audio.AudioConfig(device_name=str(device_id));
        self.speech_recognizer = speechsdk.SpeechRecognizer(speech_config=speech_config, audio_config=audio_config)

    def run(self):

        def handle_final_result(evt):
            data_string = evt.result.json
            data_json = json.loads(data_string) # {'DisplayText': 'Testing, testing.', 'Duration': 10100000, 'Id': '349903cd0a274070a62260c2d83328b9', 'Offset': 37300000, 'RecognitionStatus': 'Success'}
            trans_msg = Transcript()
            trans_msg.text = data_json["DisplayText"]
            trans_msg.duration = data_json["Duration"]/10000000
            trans_msg.offset = data_json["Offset"]/10000000
            trans_msg.timestamp = time.time()
            self.pub_transcript.publish(trans_msg)
            rospy.logdebug("{} topic: {} msg: [{}\t{}\t{}]".format(rospy.get_caller_id(), self.topic_speech_name, trans_msg.text, trans_msg.duration, trans_msg.timestamp))
            rate.sleep()

        def handle_intermediate_result(evt):
            # Example:
            # {"Duration":3800000,"Id":"729021a53d4a4c6d95b08338f566afd6","Offset":277500000,"Text":"said"}
            # {"Duration":3700000,"Id":"ffd77aa80b70443086b5f1e74fb39946","Offset":283300000,"Text":"this"}
            # {"Duration":7400000,"Id":"614610f52e3e4b129e3b141cf8427a2f","Offset":283300000,"Text":"this point"}
            # {"Duration":17300000,"Id":"085383f83a9d439babeef104176f15f8","Offset":283300000,"Text":"this point i'm saying"}
            # {"Duration":22200000,"Id":"561841199f1e4fe3aae9613ce2b01220","Offset":283300000,"Text":"this point i'm saying something"}
            data_string = evt.result.json
            data_json = json.loads(data_string) 
            trans_msg = ActiveSpeech()
            trans_msg.text = data_json["Text"]
            trans_msg.duration = data_json["Duration"]/10000000
            trans_msg.offset = data_json["Offset"]/10000000
            trans_msg.timestamp = time.time()
            self.pub_speech.publish(trans_msg)
            rospy.logdebug("{} topic: {}".format(rospy.get_caller_id(), self.topic_speech_name))
            rate.sleep()


        done = False 
        def stop_cb(evt):
            print('CLOSING on {}'.format(evt))
            speech_recognizer.stop_continuous_recognition()
            global done
            done= True
    
        # ##Connect callbacks to the events fired by the speech recognizer    
        # speech_recognizer.recognizing.connect(lambda evt: print('RECOGNIZING: {}'.format(evt)))
        # speech_recognizer.recognized.connect(lambda evt: print('RECOGNIZED: {}'.format(evt)))
        self.speech_recognizer.session_started.connect(lambda evt: print('SESSION STARTED: {}'.format(evt)))
        self.speech_recognizer.session_stopped.connect(lambda evt: print('SESSION STOPPED {}'.format(evt)))
        self.speech_recognizer.canceled.connect(lambda evt: print('CANCELED {}'.format(evt)))

        self.speech_recognizer.recognizing.connect(handle_intermediate_result)
        self.speech_recognizer.recognized.connect(handle_final_result)

        # stop continuous recognition on either session stopped or canceled events
        self.speech_recognizer.session_stopped.connect(stop_cb)
        self.speech_recognizer.canceled.connect(stop_cb)
        self.speech_recognizer.start_continuous_recognition()

        while not rospy.is_shutdown():
            time.sleep(.5)

        self.terminate()

    def terminate(self):
        if hasattr(self, "pa_handler"):
            self.pa_handler.terminate()
