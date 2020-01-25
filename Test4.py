#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jan 18 16:05:11 2020

@author: Michael Neiman
"""

import matplotlib.pyplot as plt
import numpy as np

class Room(object):
    
    def map_range_x(self, 
                    start, 
                    stop, 
                    number, 
                    y):
        
        x = np.linspace(start, stop, number)
        y = np.repeat(y, np.size(x))
        
        return np.stack((x,y), axis=-1)

    def map_range_y(self, 
                    start, 
                    stop, 
                    number, 
                    x):
        
        y = np.linspace(start, stop, number)
        x = np.repeat(x, np.size(y))
        
        return np.stack((x,y), axis=-1)

    def map_obstacle(self, 
                     left, 
                     right, 
                     points):
        
        l_x, l_y = left
        r_x, r_y = right
        res = self.map_range_x(l_x, r_x, points, l_y)
        
        return res
    
    def map_square(self, 
                   top_left, 
                   bottom_right, 
                   points):
        
        tl_x, tl_y = top_left
        br_x, br_y = bottom_right
        
        return np.concatenate((self.map_range_y(tl_y, br_y, points, tl_x),
                               self.map_range_y(tl_y, br_y, points, br_x),
                               self.map_range_x(tl_x, br_x, points, tl_y),
                               self.map_range_x(tl_x, br_x, points, br_y)))
    
    def make_room(self):
        
        walls = self.map_square((0.0,10.0), (10.0,0.0), 100)
        left_obstacle = self.map_obstacle((0.0,6.0), (6.0,6.0), 100)
        right_obstacle = self.map_obstacle((4.0,4.0), (10.0,4.0), 100)
        
        return np.concatenate((walls, right_obstacle, left_obstacle))


class Point(object):
    
    def __init__(self,
                 x,
                 y):
        self.x, self.y = x, y

class Robot(Point):
    
    def __init__(self,
                 x,
                 y,
                 heading):
        super().__init__(x, y)
        self.heading = heading
        
class Scan(object):
  
    def __init__(self, 
                 room_points,
                 robot_location,
                 robot_angle,
                 swath_number):
        
        self.room_points = room_points
        self.robot_location = robot_location
        self.robot_angle = robot_angle
        self.swath_number = swath_number
    
    def convert_room_to_polar(self):
        
        x_room, y_room = self.room_points[:, 0], self.room_points[:, 1]
        
        x_rob = np.repeat(self.robot_location[0], 
                          np.size(self.room_points, axis=0))
        y_rob = np.repeat(self.robot_location[1], 
                          np.size(self.room_points, axis=0))
        
        distance = np.sqrt(np.square(x_room-x_rob) + np.square(y_room-y_rob))
        angle = np.arctan2(y_room-y_rob, x_room-x_rob) 
        
        return np.stack((angle, distance), axis=-1)
    
    def create_swaths(self):
        
        tau = 2*np.pi
        
        angle_min_start = tau/self.swath_number/2
        angle_min_stop = tau - angle_min_start
        
        angle_min = np.linspace(angle_min_start, 
                                angle_min_stop, 
                                self.swath_number)
        
        angle_max = np.mod(angle_min + tau/self.swath_number, tau)
        
        return np.stack((angle_min, angle_max), axis=1)
    
    def closest_in_swath(self):
        
        conds = self.create_swaths()
        points = self.convert_room_to_polar()
        points_filtered = np.zeros(self.swath_number)
        
        for i in np.arange(8):
            a = np.logical_and(points[:,0] > conds[i,0],
                               points[:,0] < conds[i,1])
            points_in_swaths = np.extract(a,points)
            point_sorted = points_in_swaths[points_in_swaths[:,0].astype(int).argsort()]
            points_filtered[i] = points_sorted[0]
        
        
        return points_filtered
        

        
# Set the plot size        
plt.figure(1,figsize=(10,10))    
    
# Build the room map points
r = Room()
room = r.make_room()

# Build the robot
Robot = Robot(5.0, 1.0, 0.0)

# Build goal
Goal = Point(5.0, 9.0)

# Unzip the x-y coordinates
x, y = zip(*room)

# Plot the points
plt.scatter(x, y)  

# Plot where our robot is 
plt.scatter(Robot.x, Robot.y, color='green')

# Plot where your goal is
plt.scatter(Goal.x, Goal.y, color='red')

#  Set up a room view with a pi/4 half-angle aperture and 8 angle slices on each side of center
rv = Scan(room, (Robot.x, Robot.y), Robot.heading, 8)
b = rv.convert_room_to_polar()
a = rv.create_swaths()
b = rv.closest_in_swath()


