#!/usr/bin/env python3
# license removed for brevity

import os
import sys
import yaml
import string
import random
import numpy as np

import rospy
import rospkg
from std_msgs.msg import String
from beginner_tutorials.msg import Transcript, GameState

# import nltk
# from nlp_utils import Word2vecModule, BertModule

class GuessingProcess():
    def __init__(self):
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('beginner_tutorials')
        self.language = rospy.get_param("language")
        # self.language = "en-US"
        # nltk.data.path.append(os.path.join(package_path,'models','nltk_data'))
        # self.stop_words = nltk.corpus.stopwords.words('english')
        with open(os.path.join(package_path,'models','stop_words',self.language[:2]+'.txt'), 'r') as f: self.stop_words = [word.strip() for word in f.readlines()]
        # self.languageModel = BertModule(self.language)
        # self.languageModel = Word2vecModule(self.language, package_path)
        self.vocabulary_filepath = os.path.join(package_path,'scripts','vocabulary_alternate_guesses_'+self.language[:2]+'.txt')
        self.load_game_database()
        self.game_running = False
        self.last_guess_spoken = ""

        # Publishing nodes
        self.pub_guess_ranking = rospy.Publisher('guess_ranking', String, queue_size=10)
        # Subscribing nodes
        rospy.Subscriber('game_state', GameState, self.game_state_callback)
        rospy.Subscriber('transcript_left', Transcript, self.transcript_left_callback)
        rospy.Subscriber('transcript_right', Transcript, self.transcript_right_callback)
        rospy.Subscriber('transcript_fake_microphone', Transcript, self.transcript_left_callback)
        rospy.loginfo("{} initialized".format(rospy.get_caller_id()))
        rospy.spin()

    def reset_turn(self, turn_game_word):
        self.current_turn_game_word = turn_game_word
        # self.robot_vocab = self.possible_guesses_database[turn_game_word]['words']
        wordlist = self.possible_guesses_database[turn_game_word]['words']
        self.robot_vocab = random.sample(wordlist, len(wordlist))[:10] 
        self.vocab_freq = self.possible_guesses_database[turn_game_word]['freqs']
        self.vocab_embeddings = self.possible_guesses_database[turn_game_word]['embeddings']
        self.transcript_history = list()
        self.full_user_keywords = list()
        self.full_user_sentences = list()
        self.guess_history = list()
        self.guess_ranking = list()
        self.update_guess_ranking("")

    def transcript_left_callback(self, data):
        text = data.text
        # self.update_guess_ranking(text)

    def transcript_right_callback(self, data):
        text = data.text
        # self.update_guess_ranking(text)

    def game_state_callback(self, data):
        if data.state == "start_game":
            self.game_running = True
            self.reset_turn(data.data)
        if data.state == "new_turn":
            self.reset_turn(data.data)


    def load_game_database(self):
        with open(self.vocabulary_filepath, 'r') as stream:
            vocab_file = yaml.safe_load(stream)
        self.possible_guesses_database = dict()
        for game_word in vocab_file:
            alternative_guesses = vocab_file[game_word] #+ [game_word]
            freq_array = np.arange((len(alternative_guesses)))
            embeddings = np.zeros((len(alternative_guesses)))
            # embeddings = self.languageModel.get_embeddings(alternative_guesses)
            self.possible_guesses_database[game_word] = {'words': alternative_guesses,
                                                        'freqs': freq_array,
                                                        'embeddings': embeddings}

    def update_guess_ranking(self, new_transcription):
        if self.game_running:
            if new_transcription not in self.transcript_history and new_transcription != "":
                key_words = self.filter_transcription(new_transcription)
                self.full_user_keywords.extend(key_words)
                # self.full_user_sentences.extend(new_transcription)
            self.guess_ranking = self.compute_guess_probability()
            msg = String()
            msg.data = ';'.join(self.guess_ranking)
            self.pub_guess_ranking.publish(msg)
            rospy.logdebug("{} topic: {} msg: {}".format(rospy.get_caller_id(), 'guess_ranking', msg))


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


    def compute_guess_probability(self):
        # if self.full_user_keywords:
        #     keywords_average_emb = self.get_average_transcript_embedding(self.full_user_keywords)
        #     ## Distance from keywords to all possible guesses
        #     distance = cosine_similarity(keywords_average_emb, self.vocab_embeddings.reshape(1,-1)).reshape(1,-1)[0][0]
        #     ## Closest words of vocabulary to centroid
        #     # sorted_indexes = sorted(range(len(distance)), key=lambda k: distance[k])
        #     sorted_indexes = np.argsort(vals)#[::-1]
        #     ## Avoid words that the user said and words already guessed
        #     words_to_avoid = self.guess_history + self.full_user_keywords
        #     ranked_guess_options = [self.robot_vocab[i] for i in sorted_indexes if self.robot_vocab[i] not in words_to_avoid]
        #     ## Increase probability of Guess word with each attemp:
        #     # rush = 1 + (0.05 * (len(self.guess_history)))
        # elif self.full_user_keywords or self.full_user_sentences: 
        #     # TODO: implement the option for Bert sentence embeddings
        #     pass
        # else:
        #     ranked_guess_options = random.sample(self.robot_vocab, len(self.robot_vocab))[:10] # Instead of random.shuffle(self.robot_vocab[:10])

        ## Alternative without Language Models
        ranked_guess_options = random.sample(self.robot_vocab, len(self.robot_vocab)) # Instead of random.shuffle(self.robot_vocab[:10])
        return ranked_guess_options

    def get_average_transcript_embedding(self, word_list):
        word_list_emb = list()
        for wrd in word_list:
            word_list_emb.append(self.languageModel.get_embeddings(wrd))
        return np.array(word_list_emb).mean(axis=0).reshape(1,-1)

    def update_guess_list(self):
        if self.last_guess_spoken in self.robot_vocab: 
            self.robot_vocab.remove(self.last_guess_spoken)


if __name__ == '__main__':
    rospy.init_node('guessComputation', anonymous=False, log_level=rospy.DEBUG)
    try:
        GuessingProcess()
    except Exception as e:
        rospy.logerr(e)
        sys.exit(1)
