#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jan 18 16:05:11 2020

@author: Michael Neiman
"""

import matplotlib.pyplot as plt
import numpy as np
import math

class Room(object):
    
    def map_range_x(self, 
                    start, 
                    stop, 
                    number, 
                    y):
        
        return [[start + (stop - start) * i / number, y] \
                 for i in range(number + 1)]

    def map_range_y(self, 
                    start, 
                    stop, 
                    number, 
                    x):
        
        return [[x, start + (stop - start) * i / number] \
                 for i in range(number + 1)]

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
        res  = self.map_range_y(tl_y, br_y, points, tl_x)
        res += self.map_range_y(tl_y, br_y, points, br_x)
        res += self.map_range_x(tl_x, br_x, points, tl_y)
        res += self.map_range_x(tl_x, br_x, points, br_y)
        return res
    
    def make_room(self):
        
        walls = self.map_square((0.0,0.0), (10.0,10.0), 100)
        left_obstacle = self.map_obstacle((0.0,6.0), (6.0,6.0), 100)
        right_obstacle = self.map_obstacle((4.0,4.0), (10.0,4.0), 100)
        return walls + left_obstacle + right_obstacle


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
        
    def create_swaths(self):
        
        angles_min = [self.robot_angle + (i - 1) * (4*math.pi) / \
                      self.swath_number for i in range(self.swath_number)] 
        angles_max = [self.robot_angle + (i + 1) * (4*math.pi) / \
                      self.swath_number for i in range(self.swath_number)]        
        swaths = list(zip(angles_min, angles_max))
        
        return swaths
          
    def distance_and_angle(self, 
                           point_location):
        
        rx, ry = self.robot_location
        px, py = point_location
        dx = px - rx
        dy = py - ry
        distance = math.sqrt(dx**2 + dy**2)
        angle = np.arctan2(dy, dx)
        return (distance, angle)
    
    def closest_point_in_swath(self, 
                               angle_min, 
                               angle_max):
        
        pts = []
        for p in self.room_points:
            distance, angle = self.distance_and_angle(p)
            if angle > angle_min and angle < angle_max:
                pts.append((distance, angle))
            pts = sorted(pts)
        return pts[0]

    def lidar_observations_polar(self):
        
        pts = []
        swaths = self.create_swaths()
        for swath in swaths:
            angle_min, angle_max = swath
            pts.append(self.closest_point_in_swath(angle_min, angle_max))
        return pts

# Set the plot size        
plt.figure(figsize=(8,8))    
    
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
rv = Scan(room, (Robot.x, Robot.y), 0, 8)

# Get the lidar points for a particle
observations = rv.lidar_observations_polar()

def print_cartesian(origin, observations):
    x0, y0 = origin
    pts = []
    for obs in observations:
        angle, distance = obs
        x = x0 + distance * math.cos(angle)
        y = y0 + distance * math.sin(angle)
        pts.append((x, y))
        
    return pts

point_map = print_cartesian((5.0, 1.0), observations)

obs_x, obs_y = zip(*point_map)

plt.scatter(obs_x, obs_y, color='orange')

plt.show()