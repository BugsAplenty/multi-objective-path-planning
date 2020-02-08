#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jan 18 16:05:11 2020

@author: Michael Neiman
"""

import matplotlib.pyplot as plt
import numpy as np

# Utils:

TAU = 2*np.pi

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
'''
class Robot(Point):
    
    def __init__(self,
                 x,
                 y,
                 heading):
        super().__init__(x, y)
        self.heading = heading
'''        
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
        
        global TAU
        
        x_room, y_room = self.room_points[:, 0], self.room_points[:, 1]
        
        x_rob = np.repeat(self.robot_location[0], 
                          np.size(self.room_points, axis=0))
        y_rob = np.repeat(self.robot_location[1], 
                          np.size(self.room_points, axis=0))
        
        distance = np.sqrt(np.square(x_room-x_rob) + np.square(y_room-y_rob))
        angle = np.mod(np.arctan2(y_room-y_rob, x_room-x_rob), TAU)
        
        return np.stack((angle, distance), axis=-1)
    
    def create_swaths(self):
        
        global TAU
        
        angle_min_start = TAU/self.swath_number/2
        angle_min_stop = TAU - angle_min_start
        
        angle_min = np.linspace(angle_min_start, 
                                angle_min_stop, 
                                self.swath_number)
        
        angle_max = np.mod(angle_min + TAU/self.swath_number, TAU)
        
        return np.stack((angle_min, angle_max), axis=1)
    
    def closest_in_swath(self):
        
        global TAU
        
        swaths = self.create_swaths()
        points = self.convert_room_to_polar()
        
        points_closest = np.zeros((self.swath_number,2))
        
        for i in np.arange(self.swath_number-1):
            condition = np.logical_and(points[:,0] > swaths[i,0], 
                                       points[:,0] < swaths[i,1])
            points_filtered = points[condition]
            points_sorted = points_filtered[points_filtered[:,1].argsort()]
            if points_sorted.size == 0:
                points_closest[i] = [None, None]
            else:
                points_closest[i] = points_sorted[0]
        
        condition = np.logical_or(points[:,0] > swaths[self.swath_number-1,0],
                                  points[:,0] < swaths[self.swath_number-1,1])
        
        points_filtered = points[condition]
        points_sorted = points_filtered[points_filtered[:,1].argsort()]
        
        if points_sorted.size == 0:
            points_closest[-1] = [None, None]
        else:
            points_closest[-1] = points_sorted[0]
        
        return points_closest
    
    def safety_distance(self):
        points = self.closest_in_swath()
        points_sorted = points[points[:,1].argsort()]
        
        safety_distance_coord = points_sorted[0]
        safety_distance = np.sqrt(np.square(self.robot_location[0] - \
                                            safety_distance_coord[0]) + \
                                  np.square(self.robot_location[1] - \
                                            safety_distance_coord[1]))
        
        return safety_distance
    
    def reproject_scans_to_cartesian(self):
        points = self.closest_in_swath()
        points_cartesian = np.array([points[:,1] * np.cos(points[:,0]), 
                                     points[:,1] * np.sin(points[:,0])])
        points_cartesian = np.transpose(points_cartesian)
        
        return points_cartesian + self.robot_location
    
# Build the room map points
r = Room()
room = r.make_room()

# Build the robot
Robot = Point(5.0, 1.0)

# Build goal
Goal = Point(5.0, 9.0)

# Set the plot size        
plt.figure(1,figsize=(10,10),)    

# Unzip the x-y coordinates
x, y = zip(*room)

# Plot the points
plt.scatter(x, y)  

# Plot where our robot is 
plt.scatter(Robot.x, Robot.y, color='green')

# Plot where your goal is
plt.scatter(Goal.x, Goal.y, color='red')

"""
#  Set up a room view with a pi/4 half-angle aperture and 8 angle slices on each side of center
rv = Scan(room, (Robot.x, Robot.y), 0.0, 8)
b = rv.convert_room_to_polar()
a = rv.create_swaths()
c = rv.closest_in_swath()
o = rv.safety_distance()
d = np.transpose(np.array([c[:,1] * np.cos(c[:,0]), c[:,1] * np.sin(c[:,0])]))
e = d + np.array([Robot.x, Robot.y])
f = rv.reproject_scans_to_cartesian()
plt.scatter(f[:,0], f[:,1])

x_beams = np.linspace(0, 10, 100)
m = np.tan(a[:,0])
y_beams = Robot.y + np.tile(x_beams - Robot.x, (8,1)) * m[:, np.newaxis]
plt.scatter(np.tile(x_beams - Robot.x, (8,1))+Robot.x, y_beams)
"""




