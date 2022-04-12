# -*- coding: utf-8 -*-
"""
Created on Tue Apr 12 11:01:06 2022

@author: IntekPlus
"""

import numpy as np
import open3d as o3d
from hashtable import HashTable

class ppf():
    def __init__(self, filename):
        self.model = o3d.io.read_point_cloud(filename)
        self.tree = o3d.geometry.KDTreeFlann(self.model)
        self.norm_param = o3d.geometry.KDTreeSearchParamKNN(50)
        self.model.estimate_normals(self.norm_param)
        self.model.orient_normals_consistent_tangent_plane(k=50)
        self.model.voxel_down_sample(1.0)
        self.points = np.asarray(self.model.points)
        self.normals = np.asarray(self.model.normals)
        self.size = self.points.shape[0]
        
        #distance and angle steps
        self.d_dist = 1.0
        self.angle_steps = 20
        self.d_angle = 2*np.pi/self.angle_steps
        self.hashtable = HashTable(capacity = self.size(self.size - 1)) 
    
    def angle(self, vec1, vec2):
        ang = np.arccos(np.dot(vec1, vec2))
        return ang%np.pi
    
    def get_ppf(self, idx1, idx2):
        
        d = self.points[idx2] - self.points[idx1]
        dnorm = self.discrete_distance(np.linalg.norm(d))
        alpha = self.discrete_angle(self.angle(self.normals[idx1], d))
        beta = self.discrete_angle(self.angle(self.normals[idx2], d))
        gamma = self.discrete_angle(self.angle(self.normals[idx1], self.normals[idx2]))
        
        return np.array([dnorm, alpha, beta, gamma])
    
    def discrete_dist(self, dist):
        r = dist%self.d_dist
        d = dist//self.d_dist
        if r < self.d_dist:
            return d*self.d_dist
        else:
            return (d+1)*self.d_dist
        
    def discrete_angle(self, angle):
        r = angle%self.d_angle
        d = angle//self.d_angle
        if r < self.d_angle:
            return d*self.d_angle
        else:
            return (d+1)*self.d_angle
            
    def make_hash_table(self):
        idx = 0
        for idx1 in range(self.size):
            for idx2 in range(self.size):
                if idx1 != idx2 :            
                    ppf = self.get_ppf(idx1, idx2)
                    self.hashtable[ppf, (idx1, idx2)]
                    self.ppf_table.append(ppf)
                    print("pair: %d / %d"%(idx, self.size*self.size))
                    idx += 1
                    
                    
    
        
                    
        
        





