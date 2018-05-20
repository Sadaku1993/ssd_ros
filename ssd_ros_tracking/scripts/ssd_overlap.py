#!/usr/bin/env python
# coding:utf-8

class Rect(object):
    def __init__(self, xmin, ymin, xmax, ymax):
        self.xmin = xmin
        self.ymin = ymin
        self.xmax = xmax
        self.ymax = ymax

def range_overlap(r1_min, r1_max, r2_min, r2_max):
    overlapping = True
    if(r1_min > r2_max) and (r1_max < r2_min):
        overlapping = False
    return overlapping

def overlap(r1, r2):
    return range_overlap(r1.xmin, r1.xmax, r2.xmin, r2.xmax) and range_overlap(r1.ymin, r1.ymax, r2.ymin, r2.ymax)

def area(r1, r2):
    dx = min(r1.xmax, r2.xmax) - max(r1.xmin, r2.xmin)
    dy = min(r1.ymax, r2.ymax) - max(r1.ymin, r2.ymin)
    if (dx>=0) and (dy>=0):
        return dx * dy
    else:
        return 0.0

def overlap_rate(r1, r2, overlap_area):
    r1_area = int((r1.xmax-r1.xmin) * (r1.ymax-r1.ymin))
    r2_area = int((r2.xmax-r2.xmin) * (r2.ymax-r2.ymin))
    sum_area = int(overlap_area)
    if sum_area <= 0:
        return 0
    else:
        numerator = r1_area+r2_area-overlap_area
        rate = 1.0*overlap_area / numerator
        # print 'rate:%.2f' % (rate)
        return rate

