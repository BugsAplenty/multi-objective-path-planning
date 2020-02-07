#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb  6 20:09:13 2020

@author: nitrodubbz
"""

import matplotlib.pyplot as plt
import numpy as np
import autograd.numpy as anp
from pymoo.util.misc import stack
from pymoo.model.problem import Problem

class SafetyDistance(Problem):

    def __init__(self):
        
        super().__init__(n_var=4, 
             n_obj=2, 
             n_constr=3, 
             xl = anp.array([0,0,0,0]),
             xu = anp.array([10,10,10,10]))        
        
        def _evaluate(self, x, out):
            
            from Generator import Scan, Point, Robot, Room
            
            Robot = Robot(5.0, 1.0, 0.0)
            Goal = Point(5.0, 9.0)
            r = Room()
            Room = r.make_room()
            
            x1 = x[:,0] 
            x2 = x[:,1]
            y1 = x[:,2]
            y2 = x[:,3]
            
            d1 = np.sqrt(np.square(x1-Robot.x) + \
                         np.square(y1-Robot.y))
            
            d2 = np.sqrt(np.square(x1-x2) + np.square(y1-y2))
            
            d3 = np.sqrt(np.square(x2-Goal.x) + \
                         np.square(y2-Goal.y))
            
            d_tot = np.sum(d1,d2,d3)
            
            path1_x = np.linspace(Robot.x, x1, num=5)
            path1_y = np.linspace(Robot.y, y1, num=5)
            path2_x = np.linspace(x1, x2, num=5)
            path2_y = np.linspace(y1, y2, num=5)
            path3_x = np.linspace(x2, Goal.x, num=5)
            path3_y = np.linspace(y2, Goal.y, num=5)
            

            path_x = np.concatenate(path1_x, path2_x, path3_x)
            path_y = np.concatenate(path1_y, path2_y, path3_y)
            
            safety_distance = np.zeros(np.size(path_x))
            
            for i in np.aranage(np.size(path_x)):
                scan = Scan(Room, (path_x[i], path_y[i]), Robot.heading, 8)
                safety_distance[i] = scan.safety_distance()
                
            safety_distance = np.minimum(safety_distance)
            
            f1 = d_tot # minimize travel distance.
            f2 = -safety_distance # maximize safety distance.