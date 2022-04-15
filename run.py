# -*- coding: utf-8 -*-
"""
Created on Tue Apr 12 11:01:06 2022

@author: IntekPlus
"""

from voting import voting
from ppf import ppf


model_filename = "./BunnyData/bun000_Unstructured.pcd"
scene_filename = "./BunnyData/bun045_Unstructured.pcd"
hashtable_filename = "hashtable_bunny.txt"
model_ppf = ppf(model_filename, 0.03, hashtable_filename, True)
scene_ppf = ppf(scene_filename, 0.03)

vote = voting(model_ppf, scene_ppf, 1)
vote.vote()
t, R = vote.get_pose()
print(t, R)
