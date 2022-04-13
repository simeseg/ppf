# -*- coding: utf-8 -*-
"""
Created on Tue Apr 12 11:01:06 2022

@author: IntekPlus
"""

from voting import voting
from ppf import ppf


model_filename = "../BoltData/bolt_model.pcd"
scene_filename = "../BoltData/boltclip0.pcd"

model_ppf = ppf(model_filename, 5.0)
model_ppf.make_hash_table()
scene_ppf = ppf(scene_filename, 5.0)

vote = voting(model_ppf, scene_ppf, 100)
vote.vote()
