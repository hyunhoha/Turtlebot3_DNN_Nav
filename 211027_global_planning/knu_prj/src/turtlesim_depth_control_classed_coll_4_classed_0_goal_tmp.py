#! /usr/bin/python3
from os import stat
import rospy
# ROS Image message
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError

# from mavros_msgs.msg import *
# from geometry_msgs import TwistStamped
# OpenCV2 for saving an image
# from gazebo_msgs import ModelStates

import cv2
import numpy as np
import tensorflow as tf
from sklearn.preprocessing import normalize

from nav_msgs.msg import Odometry

from tensorflow.compat.v1 import ConfigProto
from tensorflow.compat.v1 import InteractiveSession

import tf as tft
import math


config = ConfigProto()
config.gpu_options.allow_growth = True
session = InteractiveSession(config=config)

bridge = CvBridge()
# model = tf.keras.models.load_model("/home/iasl/catkin_ws/src/knu_prj/src/DNN_model/")
# model = tf.keras.models.load_model("/home/iasl/catkin_ws/src/knu_prj/res_class_model2/")
model = tf.keras.models.load_model("/home/hyun/catkin_ws/src/knu_prj/res_collision_model_new_4_classes_0_labeled/")
control_out = 0
# pub = rospy.Publisher('ctr_out', Int16 , queue_size =10)
pub = rospy.Publisher('cmd_vel', Twist , queue_size =10)

goal = [2, 1.5]
tmp_goal = [0, 0]
my_col_prob = 0
local_flag = False
cnt = 0

def image_callback(image_data):
    # global control_out
    global twist_data
    global my_col_prob
    global local_flag
    global tmp_goal
    global cnt

    depth_image = bridge.imgmsg_to_cv2(image_data, "32FC1")
    cv_image_array = np.array(depth_image, dtype = np.dtype('f8'))
    img_re = cv2.resize(cv_image_array, dsize=(224,224), interpolation=cv2.INTER_LINEAR)
    img_norm = cv2.normalize(img_re, img_re, 0, 255, cv2.NORM_MINMAX)
    img_norm = np.uint8(img_norm)
    image = np.array(img_norm).reshape((1,224,224,1))
    pred_out = model.predict(image)
    # print("pred out : " , pred_out)
    control_out = pred_out[0][0]
    collision_prob = pred_out[1][0]
    # print("control : ", control_out)
    # print("collision prob : ", collision_prob)

    if not local_flag:
        dx = goal[0] - state[0]
        dy = goal[1] - state[1]
    else:
        dx = tmp_goal[0] - state[0]
        dy = tmp_goal[1] - state[1]

    a, b, c, d = -0.05, 3, 0.2, 3
    alpha = 0.9
    beta = 0.7
    # new_vel = (control_out[0] * a) + (control_out[2] * c) - collision_prob[0]*0.15

    # my_col_prob = collision_prob * 0.7 + my_col_prob * 0.3

    if abs(dy) < 0.1 and abs(dx) < 0.1:
        twist_data.linear.x = 0
        twist_data.angular.z = 0
        return
    elif abs(dx) <0.1 and dy>=0:
        new_ang = 3.1415/2 - state[2]
    elif abs(dx) <0.1 and dy<0:
        new_ang = -3.1415/2 - state[2]
    elif abs(dy) <0.1 and dx>=0:
        new_ang = -state[2]
    elif abs(dy) <0.1 and dx<0:
        new_ang = 3.1415 - state[2]
    else:
        tmp = math.atan2(dy, dx)#) - state[2]
        if tmp < 0:
            tmp += 6.2832
        new_ang = tmp - state[2]

    if collision_prob <= 0.8:
    # if my_col_prob <= 0.5:
        twist_data.linear.x = beta * 0.26 + (1-beta) * twist_data.linear.x    
        # print("atan : ", new_ang, "odom theta : ", state[2])
        twist_data.angular.z = 0.5* new_ang * beta + 0.5 * (1-beta) * twist_data.angular.z
        local_flag = False
    else:
        new_vel = (1 - control_out[0] + control_out[2]) * c - collision_prob[0]*0.1
        twist_data.linear.x = beta * new_vel + (1-beta) * twist_data.linear.x
        if control_out[1] >= control_out[3] and new_ang >= 0:
            twist_data.angular.z = (0.01+ control_out[0] + collision_prob[0]) * beta * 0.5 + (1-beta) * twist_data.angular.z
        elif control_out[1] < control_out[3] and new_ang < 0:
            twist_data.angular.z = -(0.01 + control_out[0] + collision_prob[0]) * beta * 0.5 + (1-beta) * twist_data.angular.z
        # elif control_out[1] >= control_out[3]:
        #     tmp_goal = [state[0] ]
        else:
            local_flag = True
            twist_data.linear.x = (1-beta) * twist_data.linear.x
            
            if cnt == 20:
                # tmp_goal = [state[0] - goal[0], state[1] - goal[1]]
                tmp_goal = [state[0], goal[1]]
            elif cnt == 40:
                tmp_goal = [goal[0], state[1]]
                cnt = 0
            cnt += 1
    pub.publish(twist_data)
    # print("cur x : ", state[0], " ", "cur y : ", state[1], "cur orientation : ", 180/3.1415*state[2], "ang_diff : ", 180/3.1415*math.atan2(goal[0]-state[0],goal[1]-state[1]))
    # print("control : ", twist_data.linear.x, twist_data.angular.z)
    
def odom_callback(odom_data):
    global state
    ori = odom_data.pose.pose.orientation

    state[0] = odom_data.pose.pose.position.x
    state[1] = odom_data.pose.pose.position.y
    tmp = tft.transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])[2]
    if tmp<0:
        tmp += 6.2832

    state[2] = tmp
    # print("Current orientation : ", state[2]*180/3.1415)


def main():
    # global twist_data
    image_topic = "/camera/depth/image_raw"
    odom_topic = "/odom"
    
    # status_topic = "/gazebo/model_states/"
    # rospy.Subscriber(status_topic, ModelStates, status_callback)
    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.Subscriber(odom_topic, Odometry, odom_callback)
    rospy.spin()

if __name__ == '__main__':
    control_out = 0
    rospy.init_node('image_listener')
    rate = rospy.Rate(10)
    state = [0, 0, 0]
    twist_data = Twist()
    twist_data.linear.y = 0
    twist_data.linear.z = 0
    twist_data.angular.x = 0
    twist_data.angular.y = 0
    # crush_state = False
    while not rospy.is_shutdown():
        main()
        rate.sleep()
