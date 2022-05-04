# -*- coding: utf-8 -*-

#given a point pair, align to a scene pair where both
#scenes pairs have a similar feature vector

import numpy as np
import matplotlib.pyplot as plt
import utils

class viz:
    def __init__(self, model, scene, rot, trans):
        self.model = np.asarray(model.points).T
        self.scene = np.asarray(scene.points).T
        self.rot = rot
        self.trans = trans
        self.transformed_cloud = self.transformation()
        # Create figure display
        self.fig = plt.figure(figsize=(10, 5))
        self.ax = self.fig.add_subplot(1, 2, 1, projection='3d')
        self.fig.suptitle("point pair features transform")
        #display 
        #plot1
        self.ax.set_xlabel('x')
        self.ax.set_ylabel('y')
        self.ax.set_zlabel('z')
        utils.axisEqual3D(self.ax)
        self.static = self.ax.scatter(self.model[0, :], self.model[1,:], self.model[2,:], marker='.',color = "blue")
        self.ax.scatter(self.scene[0,:], self.scene[1,:], self.scene[2,:], marker='.', color = "red")
        self.dynamic = self.ax.scatter(self.transformed_cloud[0,:], self.transformed_cloud[1,:], self.transformed_cloud[2,:], marker='.', color = "green")
        
        self.lines = []
        plt.show()
        
    def drawline(self, m, s):
        line, = self.ax.plot3D([self.model[0, m], self.scene[0, s]],[self.model[1, m], self.scene[1, s]],[self.model[2, m], self.scene[2, s]], color = 'c')
        #self.lines[i].set_data_3d([self.model[m,i], self.scene[s,i]],[self.model[m,i], self.scene[s,i]],[self.model[m,i], self.scene[s,i]])
        self.lines.append(line)
            
        
    def transformation(self):
        rot = np.linalg.inv(self.rot)
        trans = -rot@(self.trans.reshape(3,1))
        return rot@(self.model) + trans.reshape(3,1)
        

        
        
                