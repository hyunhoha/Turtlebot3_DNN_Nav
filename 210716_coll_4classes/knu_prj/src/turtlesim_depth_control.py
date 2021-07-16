#! /usr/bin/python3
import rospy
# ROS Image message
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
# from gazebo_msgs import ModelStates
# from collections import deque

import cv2
import numpy as np
import tensorflow as tf
from sklearn.preprocessing import normalize

bridge = CvBridge()
# model = tf.keras.models.load_model("/home/iasl/catkin_ws/src/knu_prj/src/DNN_model/")
model = tf.keras.models.load_model("/home/iasl/catkin_ws/src/knu_prj/res_lin_ang_model/")
control_out = 0
# pub = rospy.Publisher('ctr_out', Int16 , queue_size =10)
pub = rospy.Publisher('cmd_vel', Twist , queue_size =10)

# lin_vel_queue = deque([])
# def image_callback(image_data):
#     global control_out
#     depth_image = bridge.imgmsg_to_cv2(image_data, "32FC1")
#     cv_image_array = np.array(depth_image, dtype = np.dtype('f8'))
#     img_re = cv2.resize(cv_image_array, dsize=(640,480), interpolation=cv2.INTER_LINEAR)
#     img_norm = cv2.normalize(img_re, img_re, 0, 255, cv2.NORM_MINMAX)
#     img_norm = np.uint8(img_norm)
#     image = np.array(img_norm).reshape((1,480,640,1))
#     control_out = np.argmax(model.predict(image))
#     pub.publish(control_out)
#     print("control : ", control_out)

# def status_callback(status_data):
#     global crush_state

#     if status_data.twist.linear.x <= 0.01:
#         crush_state = True

def image_callback(image_data):
    global twist_data
    # global crush_state

    # if crush_state:
    #     twist_data.linear.x = -0.1
    #     twist_data.angular.z = 0
    #     pub.publish(twist_data)
    # else:
    depth_image = bridge.imgmsg_to_cv2(image_data, "32FC1")
    cv_image_array = np.array(depth_image, dtype = np.dtype('f8'))
    img_re = cv2.resize(cv_image_array, dsize=(224,224), interpolation=cv2.INTER_LINEAR)
    img_norm = cv2.normalize(img_re, img_re, 0, 255, cv2.NORM_MINMAX)
    img_norm = np.uint8(img_norm)
    image = np.array(img_norm).reshape((1,224,224,1))
    # control_out = np.argmax(model.predict(image))
    print("predicted : ", model.predict(image))
    twist_data.linear.x, twist_data.angular.z = model.predict(image)[0]
    
    # lin_vel_queue.append(twist_data.linear.x)
    # if len(lin_vel_queue) >= 2:
        
    # else:
        
    # pub.publish(control_out)
    pub.publish(twist_data)
    # print("control : ", control_out)

def main():
    # global twist_data
    image_topic = "/camera/depth/image_raw"
    # status_topic = "/gazebo/model_states/"
    # rospy.Subscriber(status_topic, ModelStates, status_callback)
    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    control_out = 0
    rospy.init_node('image_listener')
    rate = rospy.Rate(10)
    twist_data = Twist()
    twist_data.linear.y = 0
    twist_data.linear.z = 0
    twist_data.angular.x = 0
    twist_data.angular.y = 0
    # crush_state = False
    while not rospy.is_shutdown():
        main()
        rate.sleep()
