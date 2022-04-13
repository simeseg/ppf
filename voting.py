# -*- coding: utf-8 -*-

#given a point pair, align to a scene pair where both
#scenes pairs have a similar feature vector

import numpy as np
import random

class voting:
    
    def __init__(self, model_ppf, scene_ppf, sample_number):
        self.model_ppf = model_ppf
        self.scene_ppf = scene_ppf        
        self.samples = random.choices(range(self.scene_ppf.size), k = sample_number)
        self.accumulator = np.zeros([self.model_ppf.size, self.model_ppf.angle_steps])
    
    def alpha_index(self, alpha):
        return np.floor((alpha/np.pi)*self.model_ppf.angle_steps)
        
    def vote_from_reference(self, scene_idx_r):
        #reference scene point
        for scene_idx_i in range(self.scene_ppf.size):
            if scene_idx_r != scene_idx_i:
                F_s = self.scene_ppf.get_ppf(scene_idx_r, scene_idx_i)
                model_pairs = self.model_ppf.hashtable[F_s]
                for (model_idx_r, model_idx_i, alpha_model) in model_pairs:
                    #given two scene index and two model index
                    alpha = self.scene_ppf.get_alpha(scene_idx_r, scene_idx_i) - alpha_model
                    #use model_idx_r and alpha for voting
                    self.accumulator[model_idx_r, self.alpha_index(alpha)] += 1
                    
    def vote(self):
        for index in self.samples:
            self.vote_from_reference(index)
                