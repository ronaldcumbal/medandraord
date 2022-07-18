import os
import sys
import numpy as np

import gensim
# from transformers import BertModel
# from transformers import BertTokenizer
# from sklearn.metrics.pairwise import cosine_similarity,cosine_distances

class BertModule():
    def __init__(self, language, preloaded_word_game_embeddings=None):
        self.language = language
        models_path = '/home/ronald/OneDrive/models'
        # rospy.loginfo("Loading "+self.language+" BERT model")
        self.bert_model = BertModel.from_pretrained('af-ai-center/bert-base-swedish-uncased')
        self.bert_tokenizer = BertTokenizer.from_pretrained('af-ai-center/bert-base-swedish-uncased', do_lower_case=False)

    def get_embeddings(self,sentence_list):
        def mean_pooling(model_output, attention_mask):
            token_embeddings = model_output[0] #First element of model_output contains all token embeddings
            input_mask_expanded = attention_mask.unsqueeze(-1).expand(token_embeddings.size()).float()
            sum_embeddings = torch.sum(token_embeddings * input_mask_expanded, 1)
            sum_mask = torch.clamp(input_mask_expanded.sum(1), min=1e-9)
            return sum_embeddings / sum_mask
        encoded_input = self.bert_tokenizer(sentence_list, padding=True, truncation=True, max_length=128, return_tensors='pt')
        with torch.no_grad():
            model_output = self.bert_model(**encoded_input)
        #Perform pooling. In this case, mean pooling
        sentence_embeddings = mean_pooling(model_output, encoded_input['attention_mask'])
        return sentence_embeddings.numpy()#.reshape(1, -1)

    def get_similarity(self, ls1, ls2):
        """ ls1, ls2: list of sentences """
        ls1_emb = self.get_embeddings(ls1)
        ls2_emb = self.get_embeddings(ls2)
        return cosine_similarity(ls1_emb, ls2_emb)

class Word2vecModule():
    def __init__(self, language, package_path, vector_size=300, preloaded_word_game_embeddings=None):
        self.language = language
        self.vector_size = vector_size
        model_path = os.path.join(package_path,"models/Word2Vec/" + self.language[:2] + "/" + self.language + ".bin")
        # rospy.loginfo("Loading "+self.language+" Word2vec model")
        if self.language == 'en':
            self.model = gensim.models.KeyedVectors.load_word2vec_format(model_path, binary=True)
        else:
            self.model = gensim.models.Word2Vec.load(model_path)

    def get_embeddings(self, word_list):
        word_emb = list()
        for word in word_list:
            try:
                word_emb.append(np.array(self.model[word]))
            except:
                # word_emb.append(np.zeros((self.vector_size)))
                rospy.logwarn("WARNING: Embedding for word '{}' NOT FOUND in Word2Vec".format(word))
