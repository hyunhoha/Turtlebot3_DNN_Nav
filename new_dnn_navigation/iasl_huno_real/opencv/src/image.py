#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import numpy as np

# Instantiate CvBridge
bridge = CvBridge()
global current_img 
# global count
# global label
# global count
# global img_count

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
        # else:
        #     return None
    
    except CvBridgeError, e:
        print(e)
    # else:
    #     # Save your OpenCV2 image as a jpeg 
    #     # cv2.imwrite('camera_image.jpeg', cv2_img)
    #     # cv2.imwrite('camera_image.jpeg', cv_image_norm)
    #     cv2.imwrite('camera_image'+str(count)+'.jpeg', current_img)
    #     count += 1

# def store_img_and_label(num):
#     global count
#     cv2.imwrite('camera_image' + str(count)+'.jpeg', current_img)
#     f = open('label.txt', 'a')
#     f.write(str(num))
#     f.close()
#     count += 1

def main():
    global img_count
    global count

    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/camera/depth/image_raw"
    # label_topic = "/label"
    # Set up your subscriber and define its callback
    
    rospy.Subscriber(image_topic, Image, image_callback)
    # img_count += 1
    # count += 1
    # label_num = input("Type label(0~9) : ")
    # store_img_and_label(label_num)
    # rospy.Subscriber(label_topic, label, label_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    img_count = 0
    count = 1
    main()