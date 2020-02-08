#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Feb  8 13:29:17 2020

@author: neiman
"""

def intersection(s1x1, s1x2, s1y1, s1y2,
                 s2x1, s2x2, s2y1, s2y2):
    
    left = max(min(s1x1, s1x2), min(s2x1, s2x2))
    right = min(max(s1x1, s1x2), max(s2x1, s2x2))
    top = max(min(s1y1, s1y2), min(s2y1, s2y2))
    bottom = min(max(s1y1, s1y2), max(s2y1, s2y2))
        
    if top > bottom or left > right:
        return -1
        
    else:
        return 1
