#!/usr/bin/env python3.8
# license removed for brevity

import os
import sys
import time
import pyaudio
import struct
import math
import wave

import rospy
import rospkg
from beginner_tutorials.msg import GameState, WizardCommand, ActiveSpeech, Transcript

SILENCE_THRESHOLD = 0.6
#INITIAL_SPEECH_THRESHOLD = 0.010

CHANNELS = 2
FORMAT = pyaudio.paInt16
SHORT_NORMALIZE = (1.0/32768.0)
RATE = 44100  
INPUT_BLOCK_TIME = 0.05 #0.05
INPUT_FRAMES_PER_BLOCK = int(RATE*INPUT_BLOCK_TIME)

## if we get this many noisy blocks in a row, increase the threshold
#OVERSENSITIVE = 15.0/INPUT_BLOCK_TIME                    
## if we get this many quiet blocks in a row, decrease the threshold
#UNDERSENSITIVE = 120.0/INPUT_BLOCK_TIME 
## if the noise was longer than this many blocks, it's not a 'tap'
#MAX_SPEECH_BLOCKS = 0.15/INPUT_BLOCK_TIME

class SpeechActivation():

    def __init__(self, sys_path, device_name, microphone_name, INITIAL_SPEECH_THRESHOLD):
        self.INITIAL_SPEECH_THRESHOLD = INITIAL_SPEECH_THRESHOLD
        self.device_name = device_name
        self.node_name = microphone_name
        self.sys_path = sys_path
        self.topic_transc_name = 'transcript_' + self.node_name.split("_")[1]
        self.topic_speech_name = 'speech_' + self.node_name.split("_")[1]
        self.pub_transcript = rospy.Publisher(self.topic_transc_name, Transcript, queue_size=10)
        self.pub_speech = rospy.Publisher(self.topic_speech_name, ActiveSpeech, queue_size=10)
        rospy.Subscriber('wizard_commands', WizardCommand, self.wizard_commands_callback)
        self.run_id = rospy.get_param("/run_id")
        # Game parameters
        self.game_running = False
        # Microphone parameters
        self.silence_threshold = SILENCE_THRESHOLD
        self.speech_detected = False
        self.speech_stopped = True
        self.speech_detected_time = 0.0
        self.speech_detected_start = 0.0
        self.recording_frames = list()
        self.init_professional_microphone()
        rospy.loginfo("{} initialized".format(rospy.get_caller_id()))
        self.listen()
        #rospy.spin()

    def init_professional_microphone(self):
        device_index = None
        self.pa_handler = pyaudio.PyAudio()
        print("\n------------------------- {} -------------------------\n".format(self.node_name))
        for i in range(self.pa_handler.get_device_count()):
            dev = self.pa_handler.get_device_info_by_index(i)
            print('{:2} {:10} maxInChannels: {:3} maxOutChannels: {:3}'.format(i,
                dev['name'],
                dev['maxInputChannels'],
                dev['maxOutputChannels']))
            if self.device_name == dev['name']:
                print('\n{} using {} {}\n'.format(self.node_name, i, dev['name']))
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

        self.speech_threshold = self.INITIAL_SPEECH_THRESHOLD
#        self.noisycount = MAX_SPEECH_BLOCKS+1 
#        self.quietcount = 0 
#        self.errorcount = 0
        self.stream = self.open_mic_stream(device_index)

    def open_mic_stream(self, device_index):
        stream = self.pa_handler.open(format = FORMAT,
                                    channels = CHANNELS,
                                    rate = RATE,
                                    input = True,
                                    input_device_index = device_index,
                                    frames_per_buffer = INPUT_FRAMES_PER_BLOCK)
        return stream

    def stop(self):
        self.game_running = False
        self.stream.stop_stream()
        self.stream.close()
        if hasattr(self, "pa_handler"):
            self.pa_handler.terminate()

        WAVE_OUTPUT_FILENAME = os.path.join(self.sys_path, '.ros/log', self.run_id, self.node_name+".wav")
        wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(self.pa_handler.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(self.recording_frames))
        wf.close()

    def listen(self):
        while not rospy.is_shutdown():
            time.sleep(0.05)
            try:
                block = self.stream.read(INPUT_FRAMES_PER_BLOCK,exception_on_overflow = False)
                if self.game_running:
                    self.recording_frames.append(block)

                amplitude = self.get_rms(block)
                if amplitude > self.speech_threshold:
                    self.speech_started()
#                    # noisy block
#                    self.quietcount = 0
#                    self.noisycount += 1
#                    if self.noisycount > OVERSENSITIVE:
#                        # turn down the sensitivity
#                        self.speech_threshold *= 1.1
#                else:            
#                    # quiet block.
##                    if 1 <= self.noisycount <= MAX_SPEECH_BLOCKS:
#                    self.noisycount = 0
#                    self.quietcount += 1
#                    if self.quietcount > UNDERSENSITIVE:
#                        # turn up the sensitivity
#                        self.speech_threshold *= 0.9

            except IOError:
                print("Error PyAudio Speaker Dirarization")
                return

            if self.speech_detected and (time.time() - self.speech_detected_time > self.silence_threshold):
                self.speech_ended()

        self.stop()


    def get_rms(self, block):
        # RMS amplitude is defined as the square root of the 
        # mean over time of the square of the amplitude.
        # so we need to convert this string of bytes into 
        # a string of 16-bit samples...

        # we will get one short out for each 
        # two chars in the string.
        count = len(block)/2
        format = "%dh"%(count)
        shorts = struct.unpack(format, block)

        # iterate over the block.
        sum_squares = 0.0
        for sample in shorts:
            # sample is a signed short in +/- 32768. 
            # normalize it to 1.0
            n = sample * SHORT_NORMALIZE
            sum_squares += n*n
        return math.sqrt(sum_squares/count)

    def speech_started(self):
        if self.speech_stopped:
            self.speech_stopped = False
            trans_msg = ActiveSpeech()
            trans_msg.text = ""
            trans_msg.duration = 0
            trans_msg.offset = 0
            trans_msg.timestamp = time.time()
            self.speech_detected_start = time.time()
            self.pub_speech.publish(trans_msg)
            rospy.logdebug("{} topic: {} ".format(rospy.get_caller_id(), 'speech'))
        self.speech_detected = True
        self.speech_detected_time = time.time()

    def speech_ended(self):
        trans_msg = Transcript()
        trans_msg.text = ""
        trans_msg.duration = time.time() - self.speech_detected_start
        trans_msg.offset = 0
        trans_msg.timestamp = time.time()
        self.pub_transcript.publish(trans_msg)
        rospy.logdebug("{} topic: {} text: {} duration: {:.3} timestamp: {}".format(rospy.get_caller_id(), 'transcript', trans_msg.text, trans_msg.duration, trans_msg.timestamp))
        self.speech_detected = False
        self.speech_stopped = True


    def wizard_commands_callback(self, data):

        if data.command == "start_game":
            self.game_running = True

        if self.node_name == "microphone_left":
            if data.command == "left_threshold_minus":
                self.speech_threshold *= 0.9
            if data.command == "left_threshold_plus":
                self.speech_threshold *= 1.1

        if self.node_name == "microphone_right":
            if data.command == "right_threshold_minus":
                self.speech_threshold *= 0.9
            if data.command == "right_threshold_plus":
                self.speech_threshold *= 1.1
        rospy.logdebug("{} speech_threshold [default: 0.01]: {}".format(self.node_name, self.speech_threshold))

