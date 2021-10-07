#! /usr/bin/python3

# rospy for the subscriber
import rospy
from sensor_msgs.msg import Image
#include "geometry_msgs/Twist.h"
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from nav_msgs.msg import Odometry
# import math

def odom_callback(odom):
    linx = odom.twist.twist.linear.x
    angz = odom.twist.twist.angular.z
    print("Received : ", linx, angz)


def main():
    # global img_count
    # global count
    global odom_data
    rospy.init_node('odom_test')
    # Define your image topic
    # image_topic = "/camera/depth/image_raw"
    # velocity_topic = "/cmd_vel"
    odom_topic = "/odom"
    
    rospy.Subscriber(odom_topic, Odometry, odom_callback)
    # rospy.Subscriber(image_topic, Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    # img_count = 0
    # count = 1
    # current_vel = Twist()
    odom_data = Odometry()
    main()