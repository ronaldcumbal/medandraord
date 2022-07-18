#!/usr/bin/env python
# coding: utf-8

# Copyright (c) Microsoft. All rights reserved.
# Licensed under the MIT license. See LICENSE.md file in the project root for full license information.
"""
Speech recognition samples for the Microsoft Cognitive Services Speech SDK
"""

import os
import time
import wave
import string
import json
import pyaudio

import azure.cognitiveservices.speech as speechsdk

with open("azure_keys.json", "r") as infile:
     key_data = json.loads(infile.read())
SPEECH_KEY = key_data["SPEECH_KEY"]
SERVICE_REGION = key_data["SERVICE_REGION"]

def speech_recognize_once_from_mic():
    """performs one-shot speech recognition from the default microphone"""
    # <SpeechRecognitionWithMicrophone>
    speech_config = speechsdk.SpeechConfig(subscription=SPEECH_KEY, region=SERVICE_REGION)
    # Creates a speech recognizer using microphone as audio input.
    # The default language is "en-us".
    speech_recognizer = speechsdk.SpeechRecognizer(speech_config=speech_config)

    # Starts speech recognition, and returns after a single utterance is recognized. The end of a
    # single utterance is determined by listening for silence at the end or until a maximum of 15
    # seconds of audio is processed. It returns the recognition text as result.
    # Note: Since recognize_once() returns only a single utterance, it is suitable only for single
    # shot recognition like command or query.
    # For long-running multi-utterance recognition, use start_continuous_recognition() instead.
    result = speech_recognizer.recognize_once()

    # Check the result
    if result.reason == speechsdk.ResultReason.RecognizedSpeech:
        print("Recognized: {}".format(result.text))
    elif result.reason == speechsdk.ResultReason.NoMatch:
        print("No speech could be recognized")
    elif result.reason == speechsdk.ResultReason.Canceled:
        cancellation_details = result.cancellation_details
        print("Speech Recognition canceled: {}".format(cancellation_details.reason))
        if cancellation_details.reason == speechsdk.CancellationReason.Error:
            print("Error details: {}".format(cancellation_details.error_details))
    # </SpeechRecognitionWithMicrophone>

def recognize_from_microphone(device_name):
    speech_config = speechsdk.SpeechConfig(subscription=SPEECH_KEY, region=SERVICE_REGION)
    speech_config.speech_recognition_language="en-US"

    audio_config = speechsdk.audio.AudioConfig(device_name=str(device_name));
    speech_recognizer = speechsdk.SpeechRecognizer(speech_config=speech_config, audio_config=audio_config)

    print("Speak into your microphone.")
    speech_recognition_result = speech_recognizer.recognize_once_async().get()

    if speech_recognition_result.reason == speechsdk.ResultReason.RecognizedSpeech:
        print("Recognized: {}".format(speech_recognition_result.text))
    elif speech_recognition_result.reason == speechsdk.ResultReason.NoMatch:
        print("No speech could be recognized: {}".format(speech_recognition_result.no_match_details))
    elif speech_recognition_result.reason == speechsdk.ResultReason.Canceled:
        cancellation_details = speech_recognition_result.cancellation_details
        print("Speech Recognition canceled: {}".format(cancellation_details.reason))
        if cancellation_details.reason == speechsdk.CancellationReason.Error:
            print("Error details: {}".format(cancellation_details.error_details))


def recognize_from_microphone_continuous(device_name):
    speech_config = speechsdk.SpeechConfig(subscription=SPEECH_KEY, region=SERVICE_REGION)
    speech_config.speech_recognition_language="en-US"

    audio_config = speechsdk.audio.AudioConfig(device_name=str(device_name));
    speech_recognizer = speechsdk.SpeechRecognizer(speech_config=speech_config, audio_config=audio_config)


    def handle_final_result(evt):
        data_string = evt.result.json
        data_json = json.loads(data_string)
        print(data_json["DisplayText"])
        print(data_json["Duration"])
        print(data_json["Offset"])
        print(data_json["RecognitionStatus"])
        print("\n")

    done = False 
    def stop_cb(evt):
        print('CLOSING on {}'.format(evt))
        speech_recognizer.stop_continuous_recognition()
        global done
        done= True
    
    # ##Connect callbacks to the events fired by the speech recognizer    
    # speech_recognizer.recognizing.connect(lambda evt: print('RECOGNIZING: {}'.format(evt)))
    # speech_recognizer.recognized.connect(lambda evt: print('RECOGNIZED: {}'.format(evt)))
    speech_recognizer.session_started.connect(lambda evt: print('SESSION STARTED: {}'.format(evt)))
    speech_recognizer.session_stopped.connect(lambda evt: print('SESSION STOPPED {}'.format(evt)))
    speech_recognizer.canceled.connect(lambda evt: print('CANCELED {}'.format(evt)))

    speech_recognizer.recognized.connect(handle_final_result)

    # stop continuous recognition on either session stopped or canceled events
    speech_recognizer.session_stopped.connect(stop_cb)
    speech_recognizer.canceled.connect(stop_cb)
    speech_recognizer.start_continuous_recognition()

    while not done:
        time.sleep(.5)



if __name__ == '__main__':

    pyaudio_instance = pyaudio.PyAudio()
    device_index = None
    device_name = "Jabra UC VOICE 150a MS: USB Audio (hw:1,0)"
    for i in range(pyaudio_instance.get_device_count()):
        dev = pyaudio_instance.get_device_info_by_index(i)
        print('{:2} {:10} maxInputChannels: {:3} maxOutputChannels: {:3}'.format(i, dev['name'], dev['maxInputChannels'], dev['maxOutputChannels']))
        if device_name == dev['name']:
            print('\nUsing {} {}'.format(i, dev['name']))
            device_index = i
            break

    device_id = device_name[device_name.find("(")+1:device_name.find(")")]

    if device_index is None:
        raise Exception('can not find input device with {} device_name(s)'.format(device_name))


#   recognize_from_microphone(device_name=device_id)
    recognize_from_microphone_continuous(device_name=device_id)
