#! /usr/bin/python3

# rospy for the subscriber
import rospy
from sensor_msgs.msg import Image
#include "geometry_msgs/Twist.h"
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
# import math

# Instantiate CvBridge
bridge = CvBridge()
# global current_img 

# current_vel = Twist()

def velocity_callback(msg):
    global current_vel
    current_vel = msg
    if msg.linear.x >= 0.25:
        current_vel.linear.x = 0.25
    else:
        current_vel.linear.x = round(msg.linear.x, 2)

    current_vel.angular.z = round(msg.angular.z, 2)
    # print("received an image")
    # global img_count
    # global count
    # s2 = '/home/iasl/catkin_ws/src/knu_prj/label.txt'

    # try:
    #     if img_count % 20 == 0:
    #         f = open(s2, 'a')
    #         f.write(str(msg.linear.x) + " " + str(msg.angular.z) + "\n")
    
    # except:
    #     print("Except_vel")
            
def image_callback(msg):
    # print("Received an image!")
    # print(count)
    # global img_count
    global count
    global current_vel
    # print(img_count)
    # print(count)

    try:
        # if img_count % 2 == 0:
        # Hz 1/20 (Slower)
        if current_vel.linear.x < 0 or (current_vel.linear.x == 0 and current_vel.angular.z == 0):
            return None
        else:
            cv2_img = bridge.imgmsg_to_cv2(msg, "32FC1")#"bgr8")
            cv_image_array = np.array(cv2_img, dtype = np.dtype('f8'))
            cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 255, cv2.NORM_MINMAX)
            
            # current_img = cv_image_norm
            s='/home/iasl/catkin_ws/src/knu_prj/images3/camera_image'+str(count)+'.jpeg'
            s2 = '/home/iasl/catkin_ws/src/knu_prj/label3.txt'
            
            # print s
            cv2.imwrite(s, cv_image_norm)
            f = open(s2, 'a')
            f.write(str(current_vel.linear.x) + " " + str(current_vel.angular.z) + "\n")
            count += 1
            # img_count=0
            
        # img_count += 1
        # else:
        #     return None
    except:
        print("Except")

def main():
    global img_count
    global count
    

    rospy.init_node('image_listener_save')
    # Define your image topic
    image_topic = "/camera/depth/image_raw"
    velocity_topic = "/cmd_vel"
    
    rospy.Subscriber(velocity_topic, Twist, velocity_callback)
    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    img_count = 0
    count = 1
    current_vel = Twist()
    main()