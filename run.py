# -*- coding: utf-8 -*-
"""
Created on Tue Apr 12 11:01:06 2022

@author: IntekPlus
"""

from ppf import ppf

filename = "../BoltData/bolt_model.pcd"

ppair = ppf(filename)
ppair.make_ppf_table()

