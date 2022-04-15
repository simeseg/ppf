# -*- coding: utf-8 -*-

#given a point pair, align to a scene pair where both
#scenes pairs have a similar feature vector

import numpy as np
import matplotlib.pyplot as plt
import utils

class viz:
    def __init__(self, model, scene, transform):
        self.model = np.asarray(model.points).T
        self.scene = np.asarray(scene.points).T
        self.T = transform
        self.t_out = self.transform()
        # Create figure display
        self.fig = plt.figure(figsize=(10, 5))
        self.ax = self.fig.add_subplot(1, 2, 1, projection='3d')
        self.fig.suptitle("point pair features transform")
        #display 
        #plot1
        self.ax.set_xlim([self.min[0],self.max[0]])
        self.ax.set_xlabel('x')
        self.ax.set_ylabel('y')
        self.ax.set_zlabel('z')
        utils.axisEqual3D(self.ax)
        self.static = self.ax.scatter(self.model[0, :], self.model[1,:], self.model[2,:], marker='.',color = "blue")
        self.ax.scatter(self.scene[0,:], self.scene[1,:], self.scene[2,:], marker='.', color = "red")
        self.dynamic = self.ax.scatter(self.scene[0,:], self.scene[1,:], self.scene[2,:], marker='.', color = "green")
        
    def transform(self):
        return self.transform@(self.model)
        

        
        
                