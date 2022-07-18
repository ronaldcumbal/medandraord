#!/usr/bin/env python3.8
# license removed for brevity

import os
import sys
import yaml
import time
import random
import numpy as np

import rospy
import rospkg
from beginner_tutorials.msg import Transcript, GameState, WizardCommand

class Game():
    def __init__(self, condition="mediation"):
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('beginner_tutorials')
        # Load game
        self.condition = condition
        self.language = rospy.get_param("language")
        self.level_game_words_path = os.path.join(package_path,'scripts','vocabulary_game_words_'+self.language[:2]+'.txt')
        self.game_requirements = {0: {'level': 'easy', 'min_words': 5, 'min_time': 1},
                                1: {'level': 'medium', 'min_words': 5, 'min_time': 1},
                                2: {'level': 'hard', 'min_words': 1,'min_time': 1},
                                3: {'level':'end', 'max_time': 1}}
        # Publishing nodes
        self.pub_game_state = rospy.Publisher('game_state', GameState, queue_size=10)
        # Subscribing nodes
        rospy.Subscriber('wizard_commands', WizardCommand, self.wizard_callback)
        rospy.Subscriber('game_state', GameState, self.game_state_callback)
        # Game parameters
        self.game_running = False
        self.requirements_progress = 0
        self.played_word_history = dict()
        self.current_turn_word = None
        self.current_level = None
        rospy.loginfo("{} initialized".format(rospy.get_caller_id()))
        rospy.spin()

    def wizard_callback(self, data):
        command = data.command
        if command == "start_game" and not self.game_running:
            self.start_game()
        if command == "new_turn" and self.game_running:
            self.new_turn()
        if command =="starttest" and not self.game_running:
            self.start_intro_game()

    def game_state_callback(self, data):
        if data.state == "word_game_guess_spoken":
            rospy.loginfo("\n-----------------------------------------------------")
            rospy.loginfo("GAME WORD SPOKEN: {}".format(self.current_turn_word))
            rospy.loginfo("------------------------------------------------------")
    
    def start_intro_game(self):
        msg = GameState()
        msg.state = "intro_game"
        if self.language == "sv-SE":
            msg.data = "potatis"
        else:
            msg.data = "potato"
        self.pub_game_state.publish(msg)
        rospy.logdebug("{} topic: {} msg: {}".format(rospy.get_caller_id(), 'game_state', msg))

    def start_game(self):
        self.game_running = True
        self.level_start_time = time.time()
        self.game_start_time = time.time()
        self.reset_turn()
        msg = GameState()
        msg.state = "start_game"
        msg.data = self.current_turn_word
        self.pub_game_state.publish(msg)
        rospy.logdebug("{} topic: {} msg: {}".format(rospy.get_caller_id(), 'game_state', msg))

    def end_game(self):
        msg = GameState()
        msg.state = "end_game"
        msg.data = ""
        self.pub_game_state.publish(msg)
        rospy.logdebug("{} topic: {} msg: {}".format(rospy.get_caller_id(), 'game_state', msg))
        self.current_turn_word = "TheEnd"

    def new_turn(self):
        current_game_time = time.time() - self.game_start_time
        current_level_time = time.time() - self.level_start_time
        current_requirements = self.game_requirements[self.requirements_progress]
        end_state = True if current_requirements['level'] == 'end' else False
        rospy.logdebug(f"End state: {end_state}, req_progress: {self.requirements_progress > len(self.game_requirements) - 1}")

        #if self.played_word_history[current_requirements['level']] != 'end':
        if not end_state:
            current_level_played_words = self.played_word_history[current_requirements['level']]

            if current_level_time > current_requirements['min_time']*60 and len(current_level_played_words) > current_requirements['min_words']:
                self.requirements_progress += 1

        if self.requirements_progress > len(self.game_requirements) - 1 or end_state:
            self.end_game()
        else: # Go to next turn
            print("Go to next turn")
            self.reset_turn()

        # elif current_game_time > self.game_requirements[-1]['max_time']*60:
        #     self.end_game()
 
    def reset_turn(self):
        updated_level = self.game_requirements[self.requirements_progress]['level']
        self.load_level(updated_level)
        self.current_turn_word = self.get_new_word()
        msg = GameState()
        msg.data = self.current_turn_word
        msg.state = "new_turn"
        self.pub_game_state.publish(msg)
        current_game_time = time.time() - self.game_start_time
        current_level_time = time.time() - self.level_start_time
        rospy.logdebug("{} topic: {} msg: {}".format(rospy.get_caller_id(), 'game_state', msg))
        rospy.loginfo("\n---------------------- NEW TURN ----------------------")
        rospy.loginfo("Current game word: {}".format(self.current_turn_word))
        rospy.loginfo("Current level: {}".format(self.current_level))
        count, word_list = self.all_game_words()
        rospy.loginfo("Words played: ({}) - {}".format(count, word_list))
        rospy.loginfo("Time running (overall): {} mins {} seconds".format(int(current_game_time / 60), int(current_game_time % 60)))
        rospy.loginfo("Time running (level):   {} mins {} seconds".format(int(current_level_time / 60), int(current_level_time % 60)))
        rospy.loginfo("------------------------------------------------------")
        # LOG INFORMATION

    def all_game_words(self):
        text = ""
        count = 0
        for key in self.played_word_history:
            text += " ["+key+"]"
            for word in self.played_word_history[key]:
                text += " "+word
                count += 1
        return count-1, text

    def load_level(self, updated_level):
        if updated_level != self.current_level:
            with open(self.level_game_words_path, 'r') as stream:
                game_words = yaml.safe_load(stream)[updated_level]
            self.level_game_words = [word.split(',')[0].lower() for word in game_words]
            self.played_word_history[updated_level] = []
            self.level_start_time = time.time()
            self.current_level = updated_level


    def get_new_word(self):
        current_level = self.game_requirements[self.requirements_progress]['level']
        available_words = [x for x in self.level_game_words if x not in self.played_word_history[current_level]]
        new_word = random.choice(available_words)
        self.played_word_history[current_level].append(new_word)
        return new_word


if __name__ == '__main__':
    rospy.init_node('gameHandler', anonymous=False, log_level=rospy.DEBUG)
    try:
        Game()
    except Exception as e:
        rospy.logerr(e)
        sys.exit(1)
