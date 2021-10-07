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


pub = rospy.Publisher('cmd_vel', Twist , queue_size =10)


def image_callback(msg):
    global img_count
    global count
    global current_vel

    try:
        cv2_img = bridge.imgmsg_to_cv2(msg, "32FC1")
        cv_image_array = np.array(cv2_img, dtype = np.dtype('f8'))
        cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 255, cv2.NORM_MINMAX)

        for i in range(cv_image_norm.shape[0]):
            for j in range(cv_image_norm.shape[1]):
                print(i, j)
                if (cv_image_norm[i,j] == 0):
                    cv_image_norm[i,j] = 255
                    

                
        
        # current_img = cv_image_norm
        # s='/home/iasl/catkin_ws/src/knu_prj/new_images_res/camera_image'+str(count)+'.jpeg'
        # s2 = '/home/iasl/catkin_ws/src/knu_prj/new_label_res.txt'
        
        # print s
        # cv2.imwrite(s, cv_image_norm)
        # f = open(s2, 'a')
        # f.write(str(current_vel.linear.x) + " " + str(current_vel.angular.z) + "\n")
        # count += 1
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
    # rospy.Subscriber(odom_topic, Odometry, odom_callback)
    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    img_count = 0
    count = 1
    current_vel = Twist()
    main()