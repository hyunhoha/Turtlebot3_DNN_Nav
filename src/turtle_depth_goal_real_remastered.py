#! /usr/bin/python3
from os import stat

# from matplotlib.colors import TwoSlopeNorm
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
# from std_msgs.msg import Int16
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import tensorflow as tf
from sklearn.preprocessing import normalize

from nav_msgs.msg import Odometry

from tensorflow.compat.v1 import ConfigProto
from tensorflow.compat.v1 import InteractiveSession

import tf as tft
import math

import sys
import time

# import argparse

from std_msgs.msg import Float32

config = ConfigProto()
config.gpu_options.allow_growth = True
session = InteractiveSession(config=config)

bridge = CvBridge()
# model = tf.keras.models.load_model("/home/hyun/catkin_ws/src/knu_prj/res_collision_model_new_4_classes_0_labeled/")
model = tf.keras.models.load_model("/home/hyun/catkin_ws/src/knu_prj/remastered_model_220121/")
# model = tf.keras.models.load_model("/home/hyun/catkin_ws/src/knu_prj/remastered_model_22012(4)_good/")


control_out = 0
pub = rospy.Publisher('cmd_vel', Twist , queue_size =1)
pub2 = rospy.Publisher('collision', Float32, queue_size= 1)

goal = [2.0, 1.5]
# goal = [2.0, -2.0]


tmp_goal = [0, 0]
my_col_prob = 0
local_flag = False
cnt = 0
start = [0, 0]


start_time = time.time()
has_reached = False

# def parse_args(argv=None):
#     parser = argparse.ArgumentParser(
#         description='YOLACT COCO Evaluation')
#     parser.add_argument('--x',
#                         default=2.0, type=float,
#                         help='goal position x')
#     parser.add_argument('--y', default=1.5, type=float,
#                         help='goal position y')
    # parser.set_defaults(no_bar=False, display=False, resume=False, output_coco_json=False, output_web_json=False, shuffle=False,
    #                     benchmark=False, no_sort=False, no_hash=False, mask_proto_debug=False, crop=True, detect=False, display_fps=False,
    #                     emulate_playback=False)



def image_callback(image_data):
    global twist_data
    global my_col_prob
    global local_flag
    global tmp_goal
    global cnt
    global has_reached

    if has_reached:
        return

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

    # print("prediction : ", control_out)
    # print("collision prob : ", collision_prob)
    
    dx = goal[0] - state[0]
    dy = goal[1] - state[1]

    a, b, c, d = -0.05, 3, 0.2, 3
    alpha = 0.9
    beta = 0.7
    # new_vel = (control_out[0] * a) + (control_out[2] * c) - collision_prob[0]*0.15

    # my_col_prob = collision_prob * 0.7 + my_col_prob * 0.3

    if abs(dy) < 0.1 and abs(dx) < 0.1:
        twist_data.linear.x = 0
        twist_data.angular.z = 0
        print("reached")
        pub.publish(twist_data)
        print("--- %s seconds has been take to move from (%f, %f) to (%f, %f)" % (time.time() - start_time, start[0], start[1], goal[0], goal[1]))
        # sys.exit(0)
        has_reached = True
        return
    # elif abs(dx) <0.1 and dy>=0:
    #     new_ang = 3.1415/2 - state[2]
    # elif abs(dx) <0.1 and dy<0:
    #     new_ang = -3.1415/2 - state[2]
    # elif abs(dy) <0.1 and dx>=0:
    #     new_ang = -state[2]
    # elif abs(dy) <0.1 and dx<0:
    #     new_ang = 3.1415 - state[2]
    else:
        tmp = math.atan2(dy, dx)#) - state[2]
        if tmp <= 0.05:
            tmp += 6.2832
        new_ang = tmp - state[2]
        if new_ang < -3.1415:
            new_ang += 6.2832
        elif new_ang > 3.1415:
            new_ang -= 6.2832
    
    # print("goal angle : ", tmp*180/3.1415, " current : ", state[2]*180/3.1415, "Vel : ", new_ang)

    # if collision_prob <= 0.1:
    #     twist_data.linear.x = beta * 0.26 + (1-beta) * twist_data.linear.x    
    #     # print("atan : ", new_ang, "odom theta : ", state[2])
    #     # twist_data.angular.z = (collision_prob[0]+0.05) * 3 * new_ang * beta + (1-collision_prob[0]) * 0.5 * (1-beta) * twist_data.angular.z
    #     twist_data.angular.z = 0

    # control_out[1] *0.264

    # if twist_data.angular.z >= 0 and collision_prob > 0.5:
    #     control_out[0] *= (1+twist_data.angular.z)
    # elif twist_data.angular.z <0 and collision_prob > 0.5:
    #     control_out[2] *= (1-twist_data.angular.z)

    # print("prediction2 : ", control_out)


    if collision_prob <= 0.1:
    # if my_col_prob <= 0.5:
        twist_data.linear.x = beta * 0.26 + (1-beta) * twist_data.linear.x    
        # print("atan : ", new_ang, "odom theta : ", state[2])
        # twist_data.angular.z =  0.5 * new_ang * beta + (1-beta) * twist_data.angular.z

        twist_data.angular.z =  0.4 * new_ang * beta + (1-beta) * twist_data.angular.z

        # twist_data.angular.z =  (0.4-0.1 * collision_prob[0]) * new_ang * beta + (1-beta) * twist_data.angular.z



        # twist_data.angular.z = 0.5 * new_ang * beta + (1-beta) * twist_data.angular.z

        local_flag = False
        
        # beta -= 0.05
    else:
        new_vel = (1 + control_out[1]) * c - collision_prob[0]*0.4
        twist_data.linear.x = beta * new_vel + (1-beta) * twist_data.linear.x

        if control_out[0] < 0.1 and control_out[2] < 0.1:
            twist_data.angular.z = 0
        
        elif control_out[0] >= control_out[2]:# and new_ang >= 0:
            # twist_data.angular.z = (0.01+ collision_prob[0]) * beta + (1-beta) * twist_data.angular.z

            twist_data.angular.z = (0.1+ collision_prob[0] + control_out[0]) * beta * 0.7 + (1-beta) * twist_data.angular.z

            
            # twist_data.angular.z = (1+ collision_prob[0]) * control_out[0]
        elif control_out[0] < control_out[2]:# and new_ang < 0:
            twist_data.angular.z = -(0.01 + collision_prob[0] + control_out[2]) * beta * 0.7 + (1-beta) * twist_data.angular.z
            
            # twist_data.angular.z = -(1 + collision_prob[0]) * control_out[2]
                    # beta += 0.05

    if twist_data.linear.x < -0.1:
        twist_data.linear.x = -0.05
    # twist_data.angular.z = -twist_data.angular.z
    # print(collision_prob[0])

        # elif control_out[1] >= control_out[3]:
        #     tmp_goal = [state[0] ]
        # else:
        #     local_flag = True
        #     twist_data.linear.x = (1-beta) * twist_data.linear.x
            
        #     if cnt == 20:
        #         # tmp_goal = [state[0] - goal[0], state[1] - goal[1]]
        #         tmp_goal = [state[0], goal[1]]
        #     elif cnt == 40:
        #         tmp_goal = [goal[0], state[1]]
        #         cnt = 0
        #     cnt += 1

    # control_out[0] = round(control_out[0], 2)
    # control_out[1] = round(control_out[1], 2)
    # control_out[2] = round(control_out[2], 2)

    # print("prediction : ", control_out)
    # print("collision prob : ", collision_prob)


    cols = Float32()
    cols.data = collision_prob[0]
    pub.publish(twist_data)
    pub2.publish(cols)
    # print("cur x : ", state[0], " ", "cur y : ", state[1], "cur orientation : ", 180/3.1415*state[2], "ang_diff : ", 180/3.1415*math.atan2(goal[0]-state[0],goal[1]-state[1]))
    # print("control : ", twist_data.linear.x, twist_data.angular.z)
    
def odom_callback(odom_data):
    global state
    global start
    ori = odom_data.pose.pose.orientation

    state[0] = odom_data.pose.pose.position.x
    state[1] = odom_data.pose.pose.position.y
    tmp = tft.transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])[2]
    if tmp<=0.05:
        tmp += 6.2832

    state[2] = tmp

    if start[0] == 0 and start[1] == 0:
        start[0] = state[0]
        start[1] = state[1]
    # print("Current orientation : ", state[2]*180/3.1415)
    # print("current orientation : ", state[2]*180/3.1415)

def main():
    # global twist_data
    image_topic = "/camera/depth/image_raw"
    odom_topic = "/odom"

    # if has_reached:
    #     return
    # status_topic = "/gazebo/model_states/"
    # rospy.Subscriber(status_topic, ModelStates, status_callback)
    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.Subscriber(odom_topic, Odometry, odom_callback)
    rospy.spin()
    

if __name__ == '__main__':
    # has_reached = False
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
