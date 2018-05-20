#!/usr/bin/env python
# coding:utf-8

import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

import tensorflow as tf
import keras
from keras.applications.imagenet_utils import preprocess_input
from keras.backend.tensorflow_backend import set_session
from keras.models import Model
from keras.preprocessing import image

import pickle
import numpy as np
from random import shuffle
from scipy.misc import imread, imresize

from timeit import default_timer as timer
import time

import sys
sys.path.append("")

from ssd import SSD300 as SSD
from ssd_utils import BBoxUtility

from ssd_ros_msgs.msg import BoundingBox
from ssd_ros_msgs.msg import BoundingBoxArray
from amsl_recog_msgs.msg import ObjectInfoWithROI
from amsl_recog_msgs.msg import ObjectInfoArray

"""
GPUセッティング
GPU device 0
GPU memory 20%
"""
config = tf.ConfigProto(
        gpu_options = tf.GPUOptions(
            per_process_gpu_memory_fraction=0.30,
            visible_device_list="0",
            allow_growth=True
    )
)
set_session(tf.Session(config=config))


# Model load and setup
input_shape = (300, 300, 3)
class_names = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"];
NUM_CLASSES = len(class_names)
model = SSD(input_shape, num_classes=NUM_CLASSES)
model.load_weights("/home/amsl/ros_catkin_ws/src/recognition/ssd_ros/ssd_ros/scripts/weights_SSD300.hdf5")

class SSDCamera(object):
    def __init__(self, class_names, model, input_shape):
        self.class_names = class_names
        self.num_classes = len(class_names)
        self.model = model
        self.input_shape = input_shape
        self.bbox_util = BBoxUtility(self.num_classes)
        self.image_sub = rospy.Subscriber("/zed/left/image_rect_color",Image,self.imageCallback)
        self.pub = rospy.Publisher('/ssd/center_camera/objectinfo',ObjectInfoArray,queue_size=10)
        self.pub_ssd = rospy.Publisher('/ssd/center_camera/image', Image, queue_size=10)
        self.image_flag = False

        self.class_colors = []
        for i in range(0, self.num_classes):
            # This can probably be written in a more elegant manner
            hue = 255*i/self.num_classes
            col = np.zeros((1,1,3)).astype("uint8")
            col[0][0][0] = hue
            col[0][0][1] = 128 # Saturation
            col[0][0][2] = 255 # Value
            cvcol = cv2.cvtColor(col, cv2.COLOR_HSV2BGR)
            col = (int(cvcol[0][0][0]), int(cvcol[0][0][1]), int(cvcol[0][0][2]))
            self.class_colors.append(col)

    def imageCallback(self, image_msg):
        try:
            self.cv_image = CvBridge().imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeerror as e:
            print (e)
        self.image_flag = True

    def run(self, start_frame = 0, conf_thresh = 0.6):
        cv_image_height   = self.cv_image.shape[0]
        cv_image_width    = self.cv_image.shape[1]
        cv_image_channels = self.cv_image.shape[2]
        cv_image_dar      = float(cv_image_width)/float(cv_image_height)
        
        im_size = (self.input_shape[0], self.input_shape[1])
        resized = cv2.resize(self.cv_image, im_size)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        to_draw = self.cv_image.copy()

        # Use model to predict 
        inputs = [image.img_to_array(rgb)]
        tmp_inp = np.array(inputs)
        x = preprocess_input(tmp_inp)
        y = self.model.predict(x)

        results = self.bbox_util.detection_out(y)

        # BoudingBoxArray to Publish Obstacle Position
        object_array = ObjectInfoArray()
        
        if len(results) > 0 and len(results[0]) > 0:
            # Interpret output, only one frame is used 
            det_label = results[0][:, 0]
            det_conf = results[0][:, 1]
            det_xmin = results[0][:, 2]
            det_ymin = results[0][:, 3]
            det_xmax = results[0][:, 4]
            det_ymax = results[0][:, 5]

            top_indices = [i for i, conf in enumerate(det_conf) if conf >= conf_thresh]

            top_conf = det_conf[top_indices]
            top_label_indices = det_label[top_indices].tolist()
            top_xmin = det_xmin[top_indices]
            top_ymin = det_ymin[top_indices]
            top_xmax = det_xmax[top_indices]
            top_ymax = det_ymax[top_indices]

            print 'The number of detected object is %d' % (len(top_conf))
            for i in range(top_conf.shape[0]):
                xmin = int(round(top_xmin[i] * to_draw.shape[1]))
                ymin = int(round(top_ymin[i] * to_draw.shape[0]))
                xmax = int(round(top_xmax[i] * to_draw.shape[1]))
                ymax = int(round(top_ymax[i] * to_draw.shape[0]))

                class_num = int(top_label_indices[i])
                cv2.rectangle(to_draw, (xmin, ymin), (xmax, ymax), 
                        self.class_colors[class_num], 2)
                text = self.class_names[class_num] + " " + ('%.2f' % top_conf[i])
                text_top = (xmin, ymin-10)
                text_bot = (xmin + 80, ymin + 5)
                text_pos = (xmin + 5, ymin)
                cv2.rectangle(to_draw, text_top, text_bot, self.class_colors[class_num], -1)
                cv2.putText(to_draw, text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0,0,0), 1)
                
                data = ObjectInfoWithROI()
                data.Class = self.class_names[class_num]
                data.probability = top_conf[i]
                data.roi.x_offset = xmin
                data.roi.y_offset = ymin
                data.roi.height   = ymax - ymin
                data.roi.width    = xmax - xmin
                object_array.object_array.append(data)

                print '%s:%.2f xmin:%d ymin:%d xmax:%d ymax:%d' % (self.class_names[class_num], top_conf[i], xmin, ymin, xmax, ymax)

        # Calculate FPS
        self.curr_time = timer()
        self.exec_time = self.curr_time - self.prev_time
        self.prev_time = self.curr_time
        self.accum_time = self.accum_time + self.exec_time
        self.curr_fps = self.curr_fps + 1
        if self.accum_time > 1:
            self.accum_time = self.accum_time - 1
            self.fps = "FPS: " + str(self.curr_fps)
            self.curr_fps = 0

        cv2.rectangle(to_draw, (0,0), (50, 17), (255,255,255), -1)
        cv2.putText(to_draw, self.fps, (3,10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0,0,0), 1)

        # publish object data
        self.pub.publish(object_array)
        # publish image 
        pub_image = CvBridge().cv2_to_imgmsg(to_draw, "bgr8")
        self.pub_ssd.publish(pub_image)

    def main(self):
        rospy.init_node("ssd_zed")
        rate = rospy.Rate(30)

        for i in range(0, self.num_classes):
            print 'Class Num:%2d, Name:%s' % (i, self.class_names[i])
        time.sleep(2)
        
        self.accum_time = 0
        self.curr_fps = 0
        self.fps ="FPS: ??"
        self.prev_time = timer()

        while not rospy.is_shutdown():
            if self.image_flag:
                self.run(start_frame=0, conf_thresh=0.6)
            rate.sleep()

camera = SSDCamera(class_names, model, input_shape)
camera.main()
