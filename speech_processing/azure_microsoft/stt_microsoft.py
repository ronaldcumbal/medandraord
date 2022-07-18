import os
import json
import time
import pickle
import pandas as pd
import azure.cognitiveservices.speech as speechsdk

with open("azure_keys.json", "r") as infile:
     key_data = json.loads(infile.read())
SPEECH_KEY = key_data["SPEECH_KEY"]
SERVICE_REGION = key_data["SERVICE_REGION"]

def recognition_from_file(file_path):
    ''' Single-shot recognition '''
    speech_config = speechsdk.SpeechConfig(subscription=SPEECH_KEY, region=SERVICE_REGION)
    speech_config.speech_recognition_language="sv-SE"
    audio_input = speechsdk.AudioConfig(filename=file_path)
    speech_recognizer = speechsdk.SpeechRecognizer(speech_config=speech_config, audio_config=audio_input)

    result = speech_recognizer.recognize_once_async().get()
    return result.text

def print_results(result):
    if result.reason == speechsdk.ResultReason.RecognizedSpeech:
        print("Recognized: {}".format(result.text))
    elif result.reason == speechsdk.ResultReason.NoMatch:
        print("No speech could be recognized: {}".format(result.no_match_details))
    elif result.reason == speechsdk.ResultReason.Canceled:
        cancellation_details = result.cancellation_details
        print("Speech Recognition canceled: {}".format(cancellation_details.reason))
        if cancellation_details.reason == speechsdk.CancellationReason.Error:
            print("Error details: {}".format(cancellation_details.error_details))


def speech_recognize_continuous_from_file(directory, file_path, save_path):
    """performs continuous speech recognition with input from an audio file"""
    # <SpeechContinuousRecognitionWithFile>
    speech_config = speechsdk.SpeechConfig(subscription=SPEECH_KEY, region=SERVICE_REGION)
    speech_config.speech_recognition_language="sv-SE"
    speech_config.request_word_level_timestamps()
    audio_config = speechsdk.audio.AudioConfig(filename=file_path)
    # speech_recognizer = speechsdk.SpeechRecognizer(speech_config=speech_config, audio_config=audio_config)
    speech_recognizer = speechsdk.SpeechRecognizer(speech_config=speech_config,language = "sv-SE", audio_config=audio_config)

    done = False

    list_json = list()
    def handle_final_result(evt):
        data_string = evt.result.json
        data_dict = eval(data_string)
        list_json.append(data_dict)
        # print("DONE")

    def stop_cb(evt):
        """callback that signals to stop continuous recognition upon receiving an event `evt`"""
        print('CLOSING on {}'.format(evt))
        nonlocal done
        done = True

    # Connect callbacks to the events fired by the speech recognizer
    #speech_recognizer.recognizing.connect(lambda evt: print('RECOGNIZING: {}'.format(evt)))
    #speech_recognizer.recognized.connect(lambda evt: print('RECOGNIZED: {}'.format(evt)))
    #speech_recognizer.recognized.connect(lambda evt: print('JSON: {}'.format(evt.result.json)))
    speech_recognizer.recognized.connect(handle_final_result)
    speech_recognizer.session_started.connect(lambda evt: print('SESSION STARTED: {}'.format(evt)))
    # speech_recognizer.session_stopped.connect(lambda evt: print('SESSION STOPPED {}'.format(evt)))
    # speech_recognizer.canceled.connect(lambda evt: print('CANCELED {}'.format(evt)))
    
    # stop continuous recognition on either session stopped or canceled events
    speech_recognizer.session_stopped.connect(stop_cb)
    speech_recognizer.canceled.connect(stop_cb)

    # Start continuous speech recognition
    speech_recognizer.start_continuous_recognition()
    while not done:
        time.sleep(.5)
    speech_recognizer.stop_continuous_recognition()

    with open(save_path, 'wb') as f:
        pickle.dump(list_json, f, pickle.HIGHEST_PROTOCOL)


def main(audios_directory, save_directory):
    for dirpath, dirnames, filenames in os.walk(audios_directory):
        for filename in filenames:
            if filename.endswith('.wav'):
                print("Processing: ", filename)
                # local_audio_path = 'test_audio.wav'
                local_audio_path = os.path.join(dirpath,filename)
                filename = os.path.basename(os.path.normpath(local_audio_path))
                filename = os.path.splitext(filename)[0]+".p"
                save_audio_path = os.path.join(save_directory, filename)
                if not os.path.isfile(save_audio_path):  
                    speech_recognize_continuous_from_file(save_directory, local_audio_path, save_audio_path)
                    print("Saving: ", save_audio_path)


def recognize_from_microphone():
	pass

if __name__ == '__main__':

   # ============================= System Tests ============================ #
    audios_directory = 'audios'
    save_directory = 'json_transcripts_microsoft'
    main(audios_directory, save_directory)
