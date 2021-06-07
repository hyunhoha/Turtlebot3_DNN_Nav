#! /usr/bin/python3

import rospy
# ROS Image message
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import numpy as np

# Instantiate CvBridge
bridge = CvBridge()
global current_img 

def image_callback(msg):
    print("Received an image!")
    # print(count)
    global img_count
    global count
    print(img_count)
    print(count)

    try:
        if img_count % 10 == 0:
            cv2_img = bridge.imgmsg_to_cv2(msg, "32FC1")#"bgr8")
            cv_image_array = np.array(cv2_img, dtype = np.dtype('f8'))
            cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 255, cv2.NORM_MINMAX)
            
            # current_img = cv_image_norm
            s='/home/iasl/catkin_ws/src/opencv/image/camera_image'+str(count)+'.jpeg'
            # print s
            cv2.imwrite(s, cv_image_norm)
            count += 1
            img_count=0
        
        img_count += 1

    except:
        print("Except")

def main():
    global img_count
    global count

    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/camera/depth/image_raw"

    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    img_count = 0
    count = 1
    main()