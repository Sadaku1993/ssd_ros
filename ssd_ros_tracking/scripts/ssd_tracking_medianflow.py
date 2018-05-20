#!/usr/bin/env python
# coding:utf-8

import rospy
import cv2
import math
import numpy as np
import sys

from ssd_overlap import *

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ssd_ros_msgs.msg import BoundingBox
from ssd_ros_msgs.msg import BoundingBoxArray

MAX_MISS_FRAME = 10
MIN_NEW_DETECT_INTERSECTION_RATE = 0.3

class BboxClass:
    def __init__(self, data, frame, id, tracker_type):
        self.data = data
        self.id = id
        self.tracker = tracker_type
        self.n_miss_frame = 0
        self.n_collect_frame = 0
        self.bbox = (data.xmin, data.ymin, data.xmax, data.ymax)
        self.ok = self.tracker.init(frame, self.bbox)

class Callback(object):
    def __init__(self):
        self._ssd_sub = rospy.Subscriber("/ssd/BoxArray", BoundingBoxArray, self.ssdCallback)
        self._image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.imageCallback)
        self._image_pub = rospy.Publisher('/ssd/tracking', Image, queue_size=1)
        self._ssd_flag = False
        self._image_flag = False
        self._init_flag = False
        self.classes = []
        self.id = 0

    def ssdCallback(self, msg):
        self.ssd = msg
        for bbox in self.ssd.boxes:
            if bbox.Class == "person":
                b_rect = Rect(bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax)
                print "SSD xmin:%d ymin:%d xmax:%d ymax:%d" % (b_rect.xmin, b_rect.ymin, b_rect.xmax, b_rect.ymax)

        if self._image_flag:
            print "-----------------------------------------------------------------"
            # Tracker Typeの指定
            self.set_tracker()
            # Trackerの更新(追跡に失敗した場合削除)
            self.tracking()
            # 新しい検出があればそれを起点にtrackerを作成 
            self.registerTracker()
            # Visualize 
            self.show()

    def imageCallback(self, msg):
        self.image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        self.image_height = self.image.shape[0]     # y軸
        self.image_width  = self.image.shape[1]     # x軸
        self.image_channels = self.image.shape[2]
        self.frame = self.image.copy()
        self._image_flag = True

    def set_tracker(self):
        tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN']
        tracker_type = tracker_types[4]
        
        if tracker_type == 'BOOSTING':
            tracker = cv2.TrackerBoosting_create()
        if tracker_type == 'MIL':
            tracker = cv2.TrackerMIL_create()
        if tracker_type == 'KCF':
            tracker = cv2.TrackerKCF_create()
        if tracker_type == 'TLD':
            tracker = cv2.TrackerTLD_create()
        if tracker_type == 'MEDIANFLOW':
            tracker = cv2.TrackerMedianFlow_create()
        if tracker_type == 'GOTURN':
            tracker = cv2.TrackerGOTURN_create()

        self.tracker_type = tracker

    # 物体追跡
    def tracking(self):
        bbox_tmp = []
        for (i,tracker) in enumerate(self.classes):
            tracker.n_miss_frame += 1
            tracker.ok , tracker.bbox = tracker.tracker.update(self.image)
            if tracker.ok:
                p1 = (int(tracker.bbox[0]), int(tracker.bbox[1]))
                p2 = (int(tracker.bbox[2]), int(tracker.bbox[3]))
            else:
                self.classes.pop(i)
        for (i,tracker) in enumerate(self.classes):
            if MAX_MISS_FRAME < tracker.n_miss_frame: 
                self.classes.pop(i)
    
    # SSDのBboxが新しい検出か否かを判定
    def registerNewDetect(self, bbox):
        max_rate = MIN_NEW_DETECT_INTERSECTION_RATE
        id = -1
        object_flag = False
        if bbox.Class == 'person':
            b_rect = Rect(bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax)
            for tracker in self.classes:
                c_rect = Rect(tracker.bbox[0], tracker.bbox[1], tracker.bbox[2], tracker.bbox[3])
                if overlap(b_rect, c_rect):
                    overlap_area = area(b_rect, c_rect)
                    rate = overlap_rate(b_rect, c_rect, overlap_area)
                    # if max_rate < rate:
                    #     object_flag = True
                    #     tracker.n_miss_frame = 0
                    #     tracker.n_collect_frame += 1
                    #     tracker.bbox = (bbox.xmin+5, bbox.ymin+5, bbox.xmax+5, bbox.ymax+5)
                    if max_rate < rate:
                        rate = max_rate
                        object_flag = True
                        id = tracker.id
            for tracker in self.classes:
                if tracker.id == id:
                    tracker.n_miss_frame = 0
                    tracker.n_collect_frame += 1
                    tracker.bbox = (bbox.xmin+5, bbox.ymin+5, bbox.xmax+5, bbox.ymax+5)
        return object_flag

    # 新しい検出があればTrakcerを作成
    def registerTracker(self):
        for bbox in self.ssd.boxes:
            if bbox.Class == 'person':
                is_registered = self.registerNewDetect(bbox)
                if not is_registered:
                    self.classes.append(BboxClass(bbox, self.image, self.id, self.tracker_type))
                    self.id += 1
    # visualize
    def show(self):
        for tracker in self.classes:
            if tracker.ok and 2<tracker.n_collect_frame:
                p1 = (int(tracker.bbox[0]), int(tracker.bbox[1]))
                p2 = (int(tracker.bbox[2]), int(tracker.bbox[3]))
                cv2.rectangle(self.frame, p1, p2, (255,0,0), 2,1)
                p3 = (int(tracker.bbox[0]), int(tracker.bbox[1]+20))
                cv2.putText(self.frame, "ID" + str(tracker.id) , p3, cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)

        for bbox in self.ssd.boxes:
            if bbox.Class == "person":
                p1 = (int(bbox.xmin), int(bbox.ymin))
                p2 = (int(bbox.xmax), int(bbox.ymax))
                cv2.rectangle(self.frame, p1, p2, (0,0,255), 2,1)

        pub_image = CvBridge().cv2_to_imgmsg(self.frame, "bgr8")
        self._image_pub.publish(pub_image)

        # cv2.imshow("Track", self.frame)
        # cv2.waitKey(1)
                
if __name__ =='__main__':
    rospy.init_node('ssd_tracker')
    rate = rospy.Rate(20)

    callback = Callback()
    
    while not rospy.is_shutdown():
        rate.sleep()
