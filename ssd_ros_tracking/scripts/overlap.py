#!/usr/bin/env python
# coding:utf-8

import math
import numpy as np

class Point(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Rect(object):
    def __init__(self, p1, p2):
        self.left = min(p1.x, p2.x)
        self.right = max(p1.x, p2.x)
        self.bottom = min(p1.y, p2.y)
        self.top = max(p1.x, p2.x)

def range_overlap(a_min, a_max, b_min, b_max):
    overlapping = True
    if (a_min > b_max) or (a_max < b_min):
        overlapping = False
    return overlapping

def overlap(r1, r2):
    return range_overlap(r1.left, r1.right, r2.left, r2.right) and range_overlap(r1.bottom, r1.top, r2.bottom, r2.top)

def area(r1, r2):
    dx = min(r1.right, r2.right) - max(r1.left, r2.left)
    dy = min(r1.top, r2.top) - max(r1.bottom, r2.bottom)
    if (dx>=0) and (dy>=0):
        return dx * dy

if __name__ == '__main__':
    p1 = Point(1,1)
    p2 = Point(3,3)
    r1 = Rect(p1, p2)
    print "left:%d right:%d bottom:%d top:%d" % (r1.left, r1.right, r1.bottom, r1.top)
    p3 = Point(1,1)
    p4 = Point(3,3)
    r2 = Rect(p3, p4)
    print "left:%d right:%d bottom:%d top:%d" % (r2.left, r2.right, r2.bottom, r2.top)
    
    if (overlap(r1, r2)):
        print "overlap"
        print area(r1, r2)


