#!/usr/bin/env python3.8
# license removed for brevity

import os
import sys
import time
import string
import random
import numpy as np

import rospy
import rospkg
from std_msgs.msg import String
from beginner_tutorials.msg import Transcript, GameState, WizardCommand, ActiveSpeech, RobotSpeech


class Behavior():
    def __init__(self):
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('beginner_tutorials')
        self.language = rospy.get_param("language")
        if rospy.get_param("condition") == "experimental":
            self.control_condition = False
            self.experimental_condition = True
        elif rospy.get_param("condition") == "control":
            self.control_condition = True
            self.experimental_condition = False
        self.speak_control_ratio = 0.65
        # Game Parameters
        self.game_running = False
        self.game_word_spoken = False
        # nltk.data.path.append(os.path.join(package_path,'models','nltk_data'))
        # self.stop_words = nltk.corpus.stopwords.words('english')
        with open(os.path.join(package_path,'models','stop_words',self.language[:2]+'.txt'), 'r') as f: self.stop_words = [word.strip() for word in f.readlines()]
        # Speech Parameters
        self.left_accum_speech = 0.0
        self.left_accum_char = 0.0
        self.left_accum_token = 0.0
        self.left_last_speech = 0.0
        self.left_last_speaking_duration = 0.0
        self.right_accum_speech = 0.0
        self.right_accum_token = 0.0
        self.right_accum_char = 0.0
        self.right_last_speech = 0.0
        self.right_last_speaking_duration = 0.0
        self.robot_last_speech = 0.0

        self.left_turn_accum_speech = 0.0
        self.right_turn_accum_speech = 0.0
        self.left_speaking = False
        self.right_speaking = False
        self.left_right_ratio = 0.0
        self.right_left_ratio = 0.0
        self.left_speaking_start_time = 0.0
        self.right_speaking_start_time = 0.0

        # Probability distribution parameters
        self.population = np.arange(0, 1.10, 0.1)
        self.weights = [8,8,8,8,8,6,3,1,0.5,0,0]
        random.seed(a=61168821, version=2)

        # Gazing Parameters
        self.current_attendee_head = 'away'
        self.robot_last_head_motion = time.time()
        #Wizard Conditions
        self.continueIntroSecond = False
        self.continueIntro = False
        self.welcome = False
        # Publishing nodes
        self.pub_robot_speak = rospy.Publisher('robot_speak', RobotSpeech, queue_size=10)
        self.pub_robot_gesture = rospy.Publisher('robot_gesture', String, queue_size=10)
        self.pub_robot_attend = rospy.Publisher('robot_attend', String, queue_size=10)
        self.pub_game_state = rospy.Publisher('game_state', GameState, queue_size=10)
        # Subscribing nodes
        rospy.Subscriber('game_state', GameState, self.game_state_callback)
        rospy.Subscriber('guess_ranking', String, self.guess_ranking_callback)
        rospy.Subscriber('wizard_commands', WizardCommand, self.wizard_commands_callback)
        rospy.Subscriber('transcript_left', Transcript, self.transcript_left_callback)
        rospy.Subscriber('transcript_right', Transcript, self.transcript_right_callback)
        rospy.Subscriber('transcript_fake_microphone', Transcript, self.transcript_left_callback)
        rospy.Subscriber('speech_left', ActiveSpeech, self.speech_left_callback)
        rospy.Subscriber('speech_right', ActiveSpeech, self.speech_right_callback)
        rospy.Subscriber('time_ran_out', String, self.game_time_ran_out)

        rospy.loginfo("{} initialized".format(rospy.get_caller_id()))
        self.run()

    def run(self):
        while not rospy.is_shutdown():
            self.robot_execute()
            time.sleep(0.1)

    def wizard_commands_callback(self, data):
        if data.command == "say_intro":
            self.say_intro(data)
        if data.command == "continueOne":
            self.continueIntro = True
            self.test_run()
        if data.command == "continueTwo":
            self.continueIntro = False
            self.continueIntroSecond = True
            self.test_run()
        if data.command == "welcome":
            self.welcome_participants(data)
        if data.command == "welcomesecond":
            self.say_intro_cont(data)
        if data.command == "timeOut":
            self.robot_speak("", "Tiden för gissningsrundan har tagit slut", "")
        if data.command == "GissadeRätt":
            prompts = ["Gissade jag rätt?", "Fick jag rätt svar?"]
            msg = random.choice(prompts)
            self.robot_speak("", "Gissade jag rätt?", "")
        if data.command == "starttest":
            self.start_test()
        if data.command == "filler":
            prompts = ["#mm_S# jag vet inte","Jag funderar på det","Jag har det på tungan"]
            msg = random.choice(prompts)
            self.robot_speak("", msg ,"")
        if data.command == "repeat_game_word":
            #if self.game_word_spoken:
            self.robot_speak("", f"Jag gissade på {self.turn_game_word}", "")
        # Logging here because the flag output="screen" on launchfile removes logging from wizardCommands.py
        rospy.logdebug("{} topic: {} command: {}".format(rospy.get_caller_id(), 'wizard_commands', data.command))


    def game_state_callback(self, data):
        if data.state == "start_game":
            self.game_running = True
            self.reset_turn(data.data)
        if data.state == "new_turn":
            prompts = ["Nu blir det ett nytt ord!", "Nu kör vi!", "Nu blir det en ny runda!"]
            msg = random.choice(prompts)
            self.robot_speak("", msg ,"")
            self.reset_turn(data.data)
            self.game_word_spoken = False
        if data.state == "end_game":
            self.robot_speak("","Spelet har tagit slut, Tack för att ni spelade","")
            self.game_running = False
        # Logging here because the flag output="screen" on launchfile removes logging from gameHandler.py
        rospy.logdebug("{} topic: {} state: {}".format(rospy.get_caller_id(), 'game_state', data.state))

    def game_time_ran_out(self, data):
        prompts = ['Tiden för denna gissningsrunda har tagit slut', 'Nu tog tiden för gissningsrundan slut']
        msg = random.choice(prompts)
        self.robot_speak("", msg ,"")
                
    def guess_ranking_callback(self, data):
        self.guess_options = data.data.split(";")

    def reset_turn(self, turn_game_word):
        self.turn_game_word = turn_game_word
        self.turn_start_time = time.time()
        self.num_spoken_guesses = 0
        self.left_turn_accum_speech = 0.0
        self.right_turn_accum_speech = 0.0

    def transcript_left_callback(self, data):
        if self.game_running:
            filtered_tokens = self.filter_transcription(data.text)
            self.left_accum_speech += data.duration
            self.left_turn_accum_speech += data.duration
            # self.left_accum_char += sum(c.isalpha() for c in data.text)
            # self.left_accum_token += len(filtered_tokens)
            self.left_last_speech = time.time()
            self.left_last_speaking_duration = data.duration

            self.left_speaking = False
            if self.experimental_condition:
                self.left_right_ratio = self.left_accum_speech /(self.right_accum_speech+self.left_accum_speech)
                self.right_left_ratio = self.right_accum_speech /(self.right_accum_speech+self.left_accum_speech)
            rospy.loginfo("participant_update L: {} R: {} L/R: {:.2} R/L: {:.2}".format(self.left_speaking, self.right_speaking, self.left_right_ratio, self.right_left_ratio))

    def transcript_right_callback(self, data):
        if self.game_running:
            filtered_tokens = self.filter_transcription(data.text)
            self.right_accum_speech += data.duration
            self.right_turn_accum_speech += data.duration
            # self.right_accum_char += sum(c.isalpha() for c in data.text)
            # self.right_accum_token += len(filtered_tokens)
            self.right_last_speech = time.time()
            self.right_last_speaking_duration = data.duration

            self.right_speaking = False
            if self.experimental_condition:
                self.left_right_ratio = self.left_accum_speech /(self.right_accum_speech+self.left_accum_speech)
                self.right_left_ratio = self.right_accum_speech /(self.right_accum_speech+self.left_accum_speech)
            rospy.loginfo("participant_update L: {} R: {} L/R: {:.2} R/L: {:.2}".format(self.left_speaking, self.right_speaking, self.left_right_ratio, self.right_left_ratio))

    def speech_right_callback(self, data):
        if self.game_running and not self.right_speaking: 
            self.right_speaking = True
            self.right_speaking_start_time = time.time()
            self.robot_attend_to("right")
            rospy.loginfo("participant_update L: {} R: {} L/R: {:.2} R/L: {:.2}".format(self.left_speaking, self.right_speaking, self.left_right_ratio, self.right_left_ratio))

    def speech_left_callback(self, data):
        if self.game_running and not self.left_speaking: 
            self.left_speaking = True
            self.left_speaking_start_time = time.time()
            self.robot_attend_to("left")
            rospy.loginfo("participant_update L: {} R: {} L/R: {:.2} R/L: {:.2}".format(self.left_speaking, self.right_speaking, self.left_right_ratio, self.right_left_ratio))

    def filter_transcription(self, text):
        words = list()
        # for token in nltk.word_tokenize(text):
        #     if (token not in self.stop_words) and (token not in string.punctuation) and (token.isalpha()):
        #         words.append(token.lower())
        text = text.translate(str.maketrans('', '', string.punctuation))
        for token in text.split():
            if (token not in self.stop_words) and (token.isalpha()):
                words.append(token.lower())
        return words

    def robot_execute(self):
        if self.game_running:
            ## BETWEEN SPEECH

            if self.control_condition:
                self.left_right_ratio = self.speak_control_ratio # This value represent a (more or less) equal participation
                self.right_left_ratio = self.speak_control_ratio # This value represent a (more or less) equal participation

            enough_time_after_last_robot_between_speech = time.time()-self.robot_last_speech > 2.9
            enough_left_time_speaking = time.time()-self.left_speaking_start_time > 1.5
            enough_right_time_speaking = time.time()-self.right_speaking_start_time > 1.5


            if self.left_speaking:
                if enough_left_time_speaking and enough_time_after_last_robot_between_speech:
                    if random.choices(self.population, self.weights)[0] > self.left_right_ratio: #adapt the use backchannel to the ratio of speaking 
                        self.robot_attend_to("left")
                        if self.experimental_condition:
                            # Use stronger backchannels and gesture with the least speaking participant
                            if self.left_right_ratio < self.right_left_ratio:
                                self.say_backchannel("strong")
                                if random.random() > 0.35: 
                                    self.robot_gesture("Smile")
                                else:
                                    self.robot_gesture("Nod")
                            # Use notmal backchannels for the most speaking participant
                            else: 
                                self.say_backchannel("soft")
                        else: # NO nodding and normal backannels for the CONTROL condition
                            self.say_backchannel("soft")

            elif self.right_speaking:
                if enough_right_time_speaking and enough_time_after_last_robot_between_speech:
                    if random.choices(self.population, self.weights)[0] > self.right_left_ratio: #adapt the use backchannel to the ratio of speaking 
                        self.robot_attend_to("right")
                        if self.experimental_condition:
                            if self.right_left_ratio < self.left_right_ratio:
                                self.say_backchannel("strong")
                                if random.random() > 0.35: 
                                    self.robot_gesture("Smile")
                                else:
                                    self.robot_gesture("Nod")
                            else: 
                                self.say_backchannel("soft")
                        else:
                            self.say_backchannel("soft")

            ## AFTER SPEECH
            else:
                enough_time_after_last_robot_after_speech = time.time()-self.robot_last_speech > 3.5

                if self.left_last_speech > self.right_last_speech:
                    time_last_speaker = self.left_last_speech
                    last_speaking_duration = self.left_last_speaking_duration
                    last_speaker = "left"
                    self.robot_attend_to("left")
                else:
                    time_last_speaker = self.right_last_speech
                    last_speaking_duration = self.right_last_speaking_duration
                    last_speaker = "right"
                    self.robot_attend_to("right")

                if enough_time_after_last_robot_after_speech: #and (last_speaking_duration > 1.0): 
                    say_final_guess_condition = (random.random()<self.num_spoken_guesses/10.0)
                    say_some_guess_not_filler = random.random() < (time.time()-self.turn_start_time)/60 #say more guesses as the time runs out
                    min_turn_speech_furhat_guess_or_filler = self.left_turn_accum_speech>3.0 or self.right_turn_accum_speech>3.0
                    min_turn_speech_furhat_guess = self.left_turn_accum_speech>5.0 or self.right_turn_accum_speech>5.0

                    if time.time()-time_last_speaker < 5.0: # short time after last participant spoke

                        if min_turn_speech_furhat_guess_or_filler:
                            if say_some_guess_not_filler and min_turn_speech_furhat_guess and not self.game_word_spoken:
                                # Change the type of hesitation if the least speaker was the last to speak
                                if self.experimental_condition:
                                    if last_speaker == "right" and self.right_left_ratio < self.left_right_ratio:
                                        type_hesitation = "strong"
                                        self.robot_attend_to('right')
                                    elif last_speaker == "left" and self.left_right_ratio < self.right_left_ratio:
                                        type_hesitation = "strong"
                                        self.robot_attend_to('left')
                                    else: 
                                        type_hesitation = "soft"
                                        self.robot_attend_to('all')
                                else:
                                    type_hesitation = "soft"
                                    self.robot_attend_to('all')

                                if say_final_guess_condition: # Say correct word game
                                    self.say_final_guess(data=type_hesitation, word=self.turn_game_word)
                                else: # Otherwise say the top word
                                    if len(self.guess_options) > 0:
                                        self.say_alternative_guess(data=type_hesitation, word=self.guess_options[0])
                                self.num_spoken_guesses += 1
#                            else:
#                                self.say_filler()

                    else: # long time after participant spoke
                        enough_time_after_head_motion = (time.time()-self.robot_last_head_motion)>10.0
                        if time.time()-self.turn_start_time > 10.0 and enough_time_after_head_motion: # do this only after 10 seconds of the turn 
                            if random.random() < (time.time()-self.turn_start_time)/60: 
                                self.robot_attend_to(random.choice(['tablet','left_averse','right_averse']))


    def change_turn(self):
        wiz_pub = rospy.Publisher('wizard_commands', WizardCommand, queue_size=10)
        msg = WizardCommand()
        msg.command = "new_turn"
        wiz_pub.publish(msg)
        rospy.logdebug("{} topic: {} command: {}".format(rospy.get_caller_id(), 'wizard_commands', msg.command))


    def say_backchannel(self, data):
        self.robot_speak(data=data, text="", type_speech="backchannel")
        rospy.loginfo("robot_update attendee: {} L: {} R: {} backchannel: {}".format(self.current_attendee_head, self.left_speaking, self.right_speaking, data))

    def say_filler(self):
        self.robot_speak(data="", text="", type_speech="filler")

    def say_final_guess(self, data, word):
        self.robot_speak(data=data, text=word, type_speech="guess")
        msg = GameState()
        msg.state = "word_game_guess_spoken"
        self.pub_game_state.publish(msg)
        self.game_word_spoken = True
        rospy.logdebug("{} topic: {} state: {}".format(rospy.get_caller_id(), 'game_state', msg.state))
        self.update_guess_list(word)

    def say_alternative_guess(self, data, word):
        self.robot_speak(data=data, text=word, type_speech="guess")
        msg = GameState()
        msg.state = "guess_spoken"
        msg.data = word
        self.pub_game_state.publish(msg)
        rospy.logdebug("{} topic: {} state: {} data: {}".format(rospy.get_caller_id(), 'game_state', msg.state, msg.data))
        self.update_guess_list(word)

    def robot_speak(self, data, text, type_speech):
        msg = RobotSpeech()
        msg.type_speech = type_speech
        msg.text = text
        msg.data = data
        self.pub_robot_speak.publish(msg)
        rospy.logdebug("{} topic: {} type_speech: {} data: {} text: {}".format(rospy.get_caller_id(), 'robot_speak', msg.type_speech, msg.data, msg.text))
        self.robot_last_speech = time.time()

    def robot_attend_to(self, location):
        if self.current_attendee_head != location and time.time()-self.robot_last_head_motion>2.5:
            msg = String()
            msg.data = location
            self.pub_robot_attend.publish(msg)
            self.current_attendee_head = location
            rospy.logdebug("{} topic: {} data: {}".format(rospy.get_caller_id(), 'robot_attend', msg.data))
            self.robot_last_head_motion = time.time()

    def robot_gesture(self, gesture):
        # 'GazeAway', 'Nod', 'Roll']
        msg = String()
        msg.data = gesture
        self.pub_robot_gesture.publish(msg)
        rospy.logdebug("{} topic: {} data: {}".format(rospy.get_caller_id(), 'robot_gesture', msg.data))
        self.robot_last_gesture = time.time()

#    def say_next_turn(self):
#        self.robot_speak("Gissade jag rätt? Bra!")
    def say_intro(self,data):
        time.sleep(5.0)
        self.robot_speak("","#clear_throat_C#","")
        time.sleep(1.5)
        self.robot_speak("","Hej!","")
        self.robot_attend_to("right")
        time.sleep(1.0)
        self.robot_speak("","Jag är föör hatt. Välkommna!","")
        time.sleep(5.0)
        self.robot_attend_to("left")
        self.robot_speak("","Vad heter du \pau=200\  och var kommer du ifrån?","")
        time.sleep(3.0)

    def welcome_participants(self,data):
        self.robot_attend_to("right")
        self.robot_speak("","Och du då? \pau=400\  Vad heter du \pau=200\  och var kommer du ifrån?","")
        

    def say_intro_cont(self, data):
        self.robot_attend_to("left")
        self.robot_speak("","Trevligt \pau=100\  Vi ska spela spelet Med Andra Ord","")
        time.sleep(3.0)
        self.robot_attend_to("right")
        self.robot_speak("","Ni måste beskriva ett ord och jag ska försöka gissa det.","")
        time.sleep(3.5)
        self.robot_attend_to("tablet")
        self.robot_speak("","Varje ord visas på skärmen med en liten bild!","")
        time.sleep(4.0)
        self.robot_attend_to("left")
        self.robot_speak("","Ni har en minut på er att beskriva ordet \pau=500\ medan jag försöka att gissa.","")
        time.sleep(4.5)
        self.robot_attend_to("right")
        self.robot_speak("","Det finns en regel \pau=100\ ","")
        time.sleep(2.0)
        self.robot_attend_to("left")
        self.robot_speak("","Ni kan inte säga själva ordet eller varianter av ordet","")
        time.sleep(4.0)
        self.robot_attend_to("right")
        self.robot_speak("","Alla andra strategier för att beskriva ordet är tillåtna. \pau=500\ Vi ska spela i cirka 20 minuter","")
        time.sleep(6.5)
        self.robot_attend_to("left")
        self.robot_speak("","Efter spelet får ni välja eran belöning.","")
        time.sleep(4.0)
        self.robot_speak("","Nu så ska vi köra en testrunda","")
        time.sleep(4.0)
        self.robot_attend_to("right")
        self.robot_speak("","På skärmen visas nu ett ord som ni ska beskriva, \pau=500\  och jag ska gissa vilket ord som visas","")
        time.sleep(5.0)
        self.robot_attend_to("left")
        self.robot_speak("","\pau=200\ Är ni redo \pau=800\  för nu börjar testrundan","")
        time.sleep(6.0)

    def start_test(self):
        self.robot_speak("","Ordet visas nu","")
        
    def test_run(self):
        if self.continueIntro:
            self.robot_speak("","Är ordet Morot?","")
        if self.continueIntroSecond:
            self.robot_speak("","Nu vet jag \pau=400\  Ordet är Potatis","")
            time.sleep(5.0)
            self.robot_speak("","Nu är testrundan slut \pau=300\ Nu börjar det riktiga spelet. ","")
            self.robot_attend_to("all")

    def say_goodbye(self):
        text = "Det är hela tiden vi har. \pau=500\  Tack för att ni spelade med mig!"
        self.robot_speak("", text, "")
        self.robot_attend_to('all')

    ## ------------ Temporary solution for Guess Computation ------------ ##
    def update_guess_list(self, word):
        if word in self.guess_options: 
            self.guess_options.remove(word)

if __name__ == '__main__':
    rospy.init_node('behaviorModule', anonymous=False, log_level=rospy.DEBUG)
    try:
        Behavior()
    except Exception as e:
        rospy.logerr(e)
        sys.exit(1)
