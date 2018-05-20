#!/usr/bin/env python
# coding:utf-8

import rospy
import cv2
import math
import numpy as np
import sys

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ssd_ros_msgs.msg import BoundingBox
from ssd_ros_msgs.msg import BoundingBoxArray

MAX_MISS_FRAME = 10
MIN_NEW_DETECT_INTERSECTION_RATE = 0.3


class BboxClass:
    def __init__(self, data, frame, id):
        self.data = data
        self.id = id
        self.tracker = cv2.TrackerMedianFlow_create()
        self.n_miss_frame = 0
        self.n_miss_match = 0
        self.bbox = (data.xmin, data.ymin, data.xmax-data.xmin, data.ymax-data.ymin)
        self.ok = self.tracker.init(frame, self.bbox)

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

class Callback(object):
    def __init__(self):
        self._ssd_sub = rospy.Subscriber("/ssd/BoxArray", BoundingBoxArray, self.ssdCallback)
        self._image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.imageCallback)
        self._ssd_flag = False
        self._image_flag = False
        self._init_flag = False
        self.classes = []
        self.id = 0

    def ssdCallback(self,msg):
        self.ssd = msg
        self.init()
        if self._init_flag:
            self.update()
            self.tracking()

    def imageCallback(self, msg):
        self._image_flag = True
        self.image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        # if self._image_flag:
        #     cv2.imshow("Image", self.image)
        #     cv2.waitKey(1)
        self.image_height = self.image.shape[0]     # y軸
        self.image_width  = self.image.shape[1]     # x軸
        self.image_channels = self.image.shape[2]
        # print "height:%d width:%d channles:%d" % (self.image_height, self.image_width, self.image_channels)
        self._image_flag = False

    # Tracker対象が0の場合->SSDの検出結果を登録
    def init(self):
        if len(self.classes) == 0:
            print "class size 0"
            tmp = []
            for bbox in self.ssd.boxes:
                tmp.append(BboxClass(bbox, self.image, self.id))
                self.id += 1
            self.classes = tmp
            self._init_flag = True

    def update(self):
        print "######################################################################"
        for bbox in self.ssd.boxes:
            if bbox.Class == "person":
                b_rect = Rect(bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax)
                print "SSD xmin:%d ymin:%d xmax:%d ymax:%d" % (b_rect.xmin, b_rect.ymin, b_rect.xmax, b_rect.ymax)
        for track in self.classes:
            c_rect = Rect(track.bbox[0], track.bbox[1], track.bbox[0]+track.bbox[2], track.bbox[1]+track.bbox[3])
            print "Tracker id:%d miss:%d xmin:%d ymin:%d xmax:%d ymax:%d" % (track.id, track.n_miss_frame, c_rect.xmin, c_rect.ymin, c_rect.xmax, c_rect.ymax)

        
        num = 0
        for bbox in self.ssd.boxes:
            if bbox.Class == "person":
                print "SSD DETECT NUMBER:%d" % num
                # ssd座標 
                b_rect = Rect(bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax)
               
                # tracker内に保存されているかを確認
                # 新しい検出の場合新たに登録 
                registered = False
                for track in self.classes:
                    # tracker座標
                    c_rect = Rect(track.bbox[0], track.bbox[1], track.bbox[0]+track.bbox[2], track.bbox[1]+track.bbox[3])
                    # 重なり率を元に同一物体かを判定
                    if(overlap(b_rect, c_rect)):
                        # print "overlap"
                        overlap_area = area(b_rect, c_rect)
                        rate = overlap_rate(b_rect, c_rect, overlap_area)
                        if MIN_NEW_DETECT_INTERSECTION_RATE < rate:
                            print "     ID:%d rate:%.2f registered" % (track.id, rate)
                            registered = True
                            # print "     berore xmim:%d ymin:%d xmax:%d ymax:%d" % (track.bbox[0], track.bbox[1], track.bbox[0]+track.bbox[2], track.bbox[1]+track.bbox[3])
                            rect = (bbox.xmin, bbox.ymin, bbox.xmax-bbox.xmin, bbox.ymax-bbox.ymin)
                            track.bbox = rect
                            track.n_miss_frame = 0
                            
                            # print "     after xmim:%d ymin:%d xmax:%d ymax:%d" % (track.bbox[0], track.bbox[1], track.bbox[0]+track.bbox[2], track.bbox[1]+track.bbox[3])
                        else:
                            print "     ID:%d rate:%.2f failer" % (track.id, rate)
                            track.n_miss_frame += 1

                if not registered:
                    print "new detection"
                    self.classes.append(BboxClass(bbox, self.image, self.id))
                    self.id += 1
                num+=1

    def tracking(self):
        to_draw = self.image.copy()
        num = 0
        for tracker in self.classes:
            tracker.ok , tracker.bbox = tracker.tracker.update(self.image)
            if tracker.ok:
                # trackerは画角そとまで追跡できるので画角外の場合は,最大値/最小値にしておく
                # SSDとTrackerの重なり率が大きく変化してしまうため
                # xmin = tracker.bbox[0]
                # ymin = tracker.bbox[1]
                # xmax = tracker.bbox[0]+tracker.bbox[2]
                # ymax = tracker.bbox[1]+tracker.bbox[3]
                # if tracker.bbox[0]<0:
                #     xmin = 0
                # if self.image_width<tracker.bbox[0]+tracker.bbox[2]:
                #     xmax = self.image_width
                # if tracker.bbox[1]<0:
                #     ymin = 0
                # if self.image_height<tracker.bbox[1]+tracker.bbox[3]:
                #     ymax = self.image_height

                # p1 = (int(xmin), int(ymin))
                # p2 = (int(xmax), int(ymax))

                p1 = (int(tracker.bbox[0]), int(tracker.bbox[1]))
                p2 = (int(tracker.bbox[0]+tracker.bbox[2]), int(tracker.bbox[1]+tracker.bbox[3]))
                # tracker.n_miss_frame = 0
                cv2.rectangle(to_draw, p1, p2, (255,0,0), 2,1)
                cv2.putText(to_draw, "ID" + str(tracker.id) , p1, cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
            else:
                tracker.n_miss_frame += 1
                self.classes.pop(num)

            if MAX_MISS_FRAME < tracker.n_miss_frame: 
                self.classes.pop(num)
            num+=1
        cv2.imshow("Track", to_draw)
        cv2.waitKey(1)

if __name__ =='__main__':
    rospy.init_node('ssd_tracker')
    rate = rospy.Rate(20)

    callback = Callback()
    
    while not rospy.is_shutdown():
        rate.sleep()
