#!/usr/bin/env python
#coding:utf-8
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

input_shape = (300, 300, 3)

def imageCallback(image_msg):
    
    # rospy.loginfo("Subscribe Image data")
    try:
        cv_image = CvBridge().imgmsg_to_cv2(image_msg,"bgr8")
    except CvBridgeError as e:
        print (e)
    
    (rows, cols, channels) = cv_image.shape
    cv_image_width  = cols
    cv_image_height = rows
    cv_image_dar    = cv_image_width / cv_image_height;

    print "Width:%d Height:%d" % (cv_image_width, cv_image_height)

    im_size = (input_shape[0], input_shape[1])
    resized = cv2.resize(cv_image, im_size)
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    to_draw = cv2.resize(resized,(int(input_shape[0]*cv_image_dar),input_shape[1]))

    cv2.imshow("ssd_ros",cv_image)
    cv2.imshow("resized",to_draw)
    cv2.waitKey(10)

def main():
    rospy.init_node("sub_camera")

    sub_image = rospy.Subscriber("/usb_cam/image_raw",Image,imageCallback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
