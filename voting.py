# -*- coding: utf-8 -*-

#given a point pair, align to a scene pair where both
#scenes pairs have a similar feature vector

import numpy as np
import random
from hashtable import BLANK
from viz import viz

class voting:
    
    def __init__(self, model_ppf, scene_ppf, sample_number):
        self.model_ppf = model_ppf
        self.scene_ppf = scene_ppf        
        self.samples = random.choices(range(self.scene_ppf.size), k = sample_number)
        self.accumulator = np.zeros([self.model_ppf.size, self.model_ppf.angle_steps, sample_number])
        self.pose_clustering()
        self.R, self.t = self.get_pose(3)
        self.viz = viz(self.model_ppf.model, self.scene_ppf.model, self.R, self.t)  
    
    def alpha_index(self, alpha):
        return np.floor((alpha/np.pi)*self.model_ppf.angle_steps).astype(int)
        
    def vote_from_reference(self, scene_idx_r, sample_id):
        #reference scene point
        for scene_idx_i in range(self.scene_ppf.size):
            if scene_idx_r != scene_idx_i:
                F_s = self.scene_ppf.get_ppf(scene_idx_r, scene_idx_i)
                model_pairs = self.model_ppf.hashtable[F_s]
                if model_pairs is not BLANK: 
                    print(model_pairs)
                    for (model_idx_r, model_idx_i, alpha_model) in model_pairs:
                        #given two scene index and two model index
                        alpha = self.scene_ppf.get_alpha(scene_idx_r, scene_idx_i) - alpha_model
                        #use model_idx_r and alpha for voting
                        self.accumulator[model_idx_r, self.alpha_index(alpha), sample_id] += 1
                        
                    
    def pose_clustering(self):
        for sample_id, index in enumerate(self.samples):
            self.vote_from_reference(index, sample_id)
            R, t = self.get_pose(sample_id)
            
    def get_pose(self, sample_id):
        model_idx_r, alpha = np.unravel_index(self.accumulator[:,:,sample_id].argmax(), self.accumulator[:,:,sample_id].shape)
        print(model_idx_r, alpha)
        translation = self.model_ppf.points[model_idx_r]
        R2g = self.model_ppf.rotate_2g(model_idx_r)
        R_x = self.model_ppf.rotate_alpha(alpha)
        
        return R_x@R2g, translation
        
        
                