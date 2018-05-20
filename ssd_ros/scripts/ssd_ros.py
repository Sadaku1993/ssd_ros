#coding:-utf-8

import cv2
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

import sys
sys.path.append("")
from ssd import SSD300 as SSD
# from videotest import VideoTest
from ssd_camera import SSDCamera

"""
GPUセッティング
GPU device 0
GPU memory 20%
"""
config = tf.ConfigProto(
        gpu_options = tf.GPUOptions(
            per_process_gpu_memory_fraction=0.4,
            visible_device_list="0",
            allow_growth=True
    )
)
set_session(tf.Session(config=config))

input_shape = (300, 300, 3)

class_names = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"];
NUM_CLASSES = len(class_names)

model = SSD(input_shape, num_classes=NUM_CLASSES)

model.load_weights('weights_SSD300.hdf5')

# video_test = VideoTest(class_names, model, input_shape)
# video_test.run()

camera = SSDCamera(class_names, model, input_shape)
camera.main()
