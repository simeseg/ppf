# -*- coding: utf-8 -*-
"""
Created on Tue Apr 12 11:01:06 2022

@author: IntekPlus
"""
import numpy as np
from voting import voting
from ppf import ppf
from viz import viz


model_filename = "./BoltData/bolt_model.pcd"
scene_filename = "./BoltData/boltclip0.pcd"
hashtable_filename = "hashtable_bolt.txt"
model_ppf = ppf(model_filename, 5, hashtable_filename, True)
scene_ppf = ppf(scene_filename, 5)

vote = voting(model_ppf, scene_ppf, int(scene_ppf.size/5))
print(vote.R, vote.t)

