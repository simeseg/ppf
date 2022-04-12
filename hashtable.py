# -*- coding: utf-8 -*-
"""
Created on Tue Apr 12 11:01:06 2022

@author: IntekPlus
"""

BLANK = object()

class HashTable:
    def __init__(self, capacity):
        self.values = capacity * [BLANK]
        
    def __len__(self):
        return len(self.values)
    
    def _index(self, key):
        return self.hash(key)%len(self)
    
    def hash(self, key):
        d, alpha, beta, gamma = key[0], key[1], key[2], key[3]
        return int(d + 2*alpha + 3*beta + 4*gamma) 
    
    def __setitem__(self, key, value):
        self.values[self._index(key)] = value
        
    def __getitem__(self, key):
        value = self.values[self._index(key)]
        if value is BLANK:
            raise KeyError(key)
        return value
    
    def __contains__(self, key):
        try:
            self[key]
        except KeyError:
            return False
        else:
            return True
        
    def get(self, key, default=None):
        try:
            return self[key]
        except KeyError:
            return default
        
    def __delitem__(self, key):
        if key in self:
            self[key] = BLANK
        else:
            raise KeyError(key)
        
        
        