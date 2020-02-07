#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Feb  7 20:39:21 2020

@author: neiman
"""

def _evaluate(self, x, out, *args, **kwargs):
            
        from Generator import Scan, Point, Room
            
        Robot = Point(5.0, 1.0)
        Goal = Point(5.0, 9.0)
        r = Room()
        Room = r.make_room()
        
        wall_left_x = np.array([0.0, 6.0])
        wall_left_y = np.array([6.0, 6.0])
        
        wall_right_x = np.array([4.0, 10.0])
        wall_right_y = np.array([4.0, 4.0])
            
        x1 = x[:,0] 
        x2 = x[:,1]
        y1 = x[:,2]
        y2 = x[:,3]
            
        d1 = np.sqrt(np.square(x1-Robot.x) + np.square(y1-Robot.y))
            
        d2 = np.sqrt(np.square(x1-x2) + np.square(y1-y2))
            
        d3 = np.sqrt(np.square(x2-Goal.x) + np.square(y2-Goal.y))
            
        d_tot = d1 + d2 + d3
            
        path1_x = np.linspace(np.transpose(np.repeat(Robot.x, np.size(x1))),
                                           x1, 
                                           num=5)
        path1_y = np.linspace(np.transpose(np.repeat(Robot.y, np.size(y1))),
                              y1,
                              num=5)
        path2_x = np.linspace(x1, x2, num=5)
        path2_y = np.linspace(y1, y2, num=5)
        path3_x = np.linspace(x2, 
                              np.transpose(np.repeat(Goal.x, np.size(x2))), 
                              num=5)
        path3_y = np.linspace(y2, 
                              np.transpose(np.repeat(Goal.y, np.size(y2))),
                              num=5)
            
        path_x = np.concatenate((path1_x, path2_x, path3_x), axis=1)
        path_y = np.concatenate((path1_y, path2_y, path3_y), axis=1)
            
        safety_distance = np.zeros(np.size(path_x))
            
        for i in np.arange(np.size(path_x)):
            scan = Scan(Room, (path_x[i], path_y[i]), 0.0, 8)
            safety_distance[i] = scan.safety_distance()
                
        safety_distance = np.minimum(safety_distance)
            
        f1 = d_tot # minimize travel distance.
        f2 = -safety_distance # maximize safety distance.
        
        top1 = np.minimum(np.maximum(wall_right_y[0], wall_right_y[1]),
                          np.maxmimum(Robot.y, y1))
        top2 = np.minimum(np.maximum(wall_left_y[0], wall_left_y[1]),
                          np.maximum(Robot.y, y1))
        top3 = np.minimum(np.maximum(wall_right_y[0], wall_right_y[1]),
                          np.maximum(y1, y2))
        top4 = np.minimum(np.maximum(wall_left_y[0], wall_left_y[1]),
                          np.maximum(y1, y2))
        top5 = np.minimum(np.maximum(wall_right_y[0], wall_right_y[1]),
                          np.maximum(y2, Goal.y))
        top6 = np.minimum(np.maximum(wall_left_y[0], wall_left_y[1]),
                          np.maximum(y2, Goal.y))
        
        bottom1 = np.maximum(np.minimum(wall_right_y[0], wall_right_y[1]),
                             np.minimum(Robot.y, y1))
        bottom2 = np.maximum(np.minimum(wall_left_y[0], wall_left_y[1]),
                             np.minimum(Robot.y, y1)) 
        bottom3 = np.maximum(np.minimum(wall_right_y[0], wall_right_y[1]),
                             np.minimum(y1, y2)) 
        bottom4 = np.maximum(np.minimum(wall_left_y[0], wall_left_y[1]),
                             np.minimum(y1, y2)) 
        bottom5 = np.maximum(np.minimum(wall_right_y[0], wall_right_y[1]),
                             np.minimum(y2, Goal.y)) 
        bottom6 = np.maximum(np.minimum(wall_left_y[0], wall_left_y[1]),
                             np.minimum(y2, Goal.y)) 
        
        
        # Constraint for path1
        # With left obstacle
        g1_left = bottom1 - top1
        # With right obstacle
        g1_right = bottom2 - top2
        
        # Constraint for path2
        # With left obstacle
        g2_left = bottom3 - top3
        # With right obstacle
        g2_right = bottom4 - top4
        
        # constraint for path3
        # With left obstacle
        g3_left = bottom5 - top5
        # With right obstacle
        g3_right = bottom6 - top6
            
        out["F"] = anp.column_stack([f1, f2])
        out["G"] = anp.column_stack([g1_left, 
           g1_right, 
           g2_left, 
           g2_right,
           g3_left, 
           g3_right])