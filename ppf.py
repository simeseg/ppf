# -*- coding: utf-8 -*-
"""
Created on Tue Apr 12 11:01:06 2022

@author: IntekPlus
"""

import numpy as np
import open3d as o3d
from hashtable import HashTable, BLANK

class ppf():
    def __init__(self, cloud_filename, voxel_size, hashtable_filename = None, make_table = False):
        self.input = o3d.io.read_point_cloud(cloud_filename)
        self.norm_param = o3d.geometry.KDTreeSearchParamRadius(2*voxel_size)
        self.input.estimate_normals(self.norm_param)
        self.input.orient_normals_consistent_tangent_plane(k=50)
        self.model = self.input.voxel_down_sample(voxel_size)
        self.points = np.asarray(self.model.points)
        self.normals = np.asarray(self.model.normals)
        self.tree = o3d.geometry.KDTreeFlann(self.model)
        self.size = self.points.shape[0]
        
        #distance and angle steps
        self.distance_sampling_rate = 0.05
        self.model_diameter = np.max(np.max(self.points, axis=0) - np.min(self.points, axis=0))
        self.d_dist = self.distance_sampling_rate*self.model_diameter
        self.angle_steps = 30
        self.d_angle = 2*np.pi/self.angle_steps
        
        if make_table:
            try:
                self.hashtable_filename = hashtable_filename
                self.hashtable = HashTable(capacity = (self.angle_steps)**3) 
                self.read_and_parse_hash_file()
                print("Read ", self.hashtable_filename)
            except OSError:
                print("Could not open/read hashtable file ", self.hashtable_filename)
                self.make_and_save_hash_table()
        
    def read_and_parse_hash_file(self):
        with open(self.hashtable_filename, 'r') as f:
            contents = f.readlines()
            for i, item in enumerate(contents):
                if item == 'None':
                    self.hashtable.values[i] = BLANK
                else:
                    self.hashtable.values[i] = [tuple([int(trip.split()[0]), int(trip.split()[1]), float(trip.split()[2])]) for trip in item.split(";")[:-1]]
        
    def make_and_save_hash_table(self):
        self.make_hash_table()
        print(self.hashtable.values)
        with open(self.hashtable_filename, 'w') as f:
            for i, item in enumerate(self.hashtable.values):
                if item is BLANK:
                    f.write("None\n")
                else:
                    for trip in item:
                        f.write("%d %d %f; " % (trip[0], trip[1], trip[2]))
                    f.write("\n")
                

    
    def vis(self):
        o3d.visualization.draw_geometries([self.model])
        
    def angle(self, vec1, vec2):
        vec1 = vec1/np.linalg.norm(vec1)
        vec2 = vec2/np.linalg.norm(vec2)
        ang = np.arccos(np.dot(vec1, vec2))
        return ang%np.pi
        #cross =  np.linalg.norm(np.cross(vec1, vec2))
        #dot = np.dot(vec1, vec2)
        #print("cross", cross, dot)
        #return np.arctan2(cross, dot)%np.pi
    
    def get_ppf(self, idx1, idx2):
        
        d = self.points[idx2] - self.points[idx1]
        dnorm = self.discrete_distance(np.linalg.norm(d))
        alpha = self.discrete_angle(self.angle(self.normals[idx1], d))
        beta = self.discrete_angle(self.angle(self.normals[idx2], d))
        gamma = self.discrete_angle(self.angle(self.normals[idx1], self.normals[idx2]))
        
        return np.array([dnorm, alpha, beta, gamma])
    
    def discrete_distance(self, dist):
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
        print(self.hashtable.values)
        for idx1 in range(self.size):
            for idx2 in range(self.size):
                if idx1 != idx2 :            
                    ppf = self.get_ppf(idx1, idx2)
                    self.hashtable[ppf] = (idx1, idx2, self.get_alpha(idx1, idx2))
                    print("pair: %d / %d"%(idx, self.size*self.size))
                    idx += 1
                    
    def rodrigues(self, angle, axis):
        if np.linalg.norm(axis) != 0:
            axis /= np.linalg.norm(axis)
        #skew symmetric form
        s_axis = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        R = np.eye(3) + np.sin(angle)*s_axis + (1- np.cos(angle))*s_axis@s_axis       
        return R
        
    def rotate_2g(self, idx_r):
        v1 = self.normals[idx_r] 
        v2 = np.array([1,0,0])
        #rotation axis and matrix
        angle = np.arccos(min(v1.dot(v2), 1))
        axis = -np.cross(v1,v2)
        return self.rodrigues(angle, axis)
        
    def get_alpha(self, idx_r, idx_i):
        pt_idx_i = self.points[idx_i] - self.points[idx_r]              #translate idx_i to origin
        R2g = self.rotate_2g(idx_r)                                     #rotate idx_r normal to x axis
        pt_idx_i = R2g@(pt_idx_i.reshape(3,1))                          #rotate idx_i point similarly
        pt_idx_i[0] = 0                                                 #project on y-z plane
        pt_idx_i /= np.linalg.norm(pt_idx_i)                            #normalize
        return np.arccos(pt_idx_i.reshape(3).dot(np.array([0,1,0])))    #find the angle to +ve y-axis    
    
    def rotate_alpha(self, alpha):
        #rotation axis and matrix
        angle = alpha
        axis = np.array([1.0,0.0,0.0])
        return self.rodrigues(angle, axis)                                 
        

