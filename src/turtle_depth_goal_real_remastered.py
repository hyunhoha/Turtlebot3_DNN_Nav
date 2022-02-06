#! /usr/bin/python3

from os import stat

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
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

from std_msgs.msg import Float32

config = ConfigProto()
config.gpu_options.allow_growth = True
session = InteractiveSession(config=config)

bridge = CvBridge()

# 모델 load
model = tf.keras.models.load_model("/home/hyun/catkin_ws/src/knu_prj/remastered_model_220121/")

control_out = 0

pub = rospy.Publisher('cmd_vel', Twist , queue_size =1)
pub2 = rospy.Publisher('collision', Float32, queue_size= 1)

# 목표 위치
goal = [2.0, 1.5]

# 시작 위치
start = [0, 0]


start_time = time.time()
has_reached = False

# 로봇의 현재 상태를 갱신한다 (로봇의 위치, 보고 있는 방향)
def odom_callback(odom_data):
    global state
    global start
    ori = odom_data.pose.pose.orientation

    # state = [x, y, 현재 보고있는 방향(direction)] 의 크기 3짜리 배열임.
    state[0] = odom_data.pose.pose.position.x
    state[1] = odom_data.pose.pose.position.y
    tmp = tft.transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])[2]
    if tmp<=0.05:
        tmp += 6.2832

    state[2] = tmp

    if start[0] == 0 and start[1] == 0:
        start[0] = state[0]
        start[1] = state[1]


def image_callback(image_data):
    global twist_data
    global has_reached

    # 도착 했으면 움직이지 말고 멈춘다. 
    if has_reached:
        return

    # 들어온 이미지를 모델에 넣어서 분류 확률이랑 충돌 확률을 얻는다.
    depth_image = bridge.imgmsg_to_cv2(image_data, "32FC1")
    cv_image_array = np.array(depth_image, dtype = np.dtype('f8'))
    img_re = cv2.resize(cv_image_array, dsize=(224,224), interpolation=cv2.INTER_LINEAR)
    img_norm = cv2.normalize(img_re, img_re, 0, 255, cv2.NORM_MINMAX)
    img_norm = np.uint8(img_norm)
    image = np.array(img_norm).reshape((1,224,224,1))
    pred_out = model.predict(image)


    # 분류된 [좌회전, 직진, 우회전 확률]과, [충돌확률(0~1사이)]
    control_out = pred_out[0][0]
    collision_prob = pred_out[1][0]

    # print("prediction : ", control_out)
    # print("collision prob : ", collision_prob)
    
    # 목표 위치랑 지금 로봇의 위치 차이
    dx = goal[0] - state[0]
    dy = goal[1] - state[1]

    # 하이퍼파라미터
    a, b, c, d = -0.05, 3, 0.2, 3
    alpha = 0.9
    beta = 0.7

    # 목표 지점에 도달한 경우.
    if abs(dy) < 0.1 and abs(dx) < 0.1:
        twist_data.linear.x = 0
        twist_data.angular.z = 0
        print("reached")
        pub.publish(twist_data)
        print("--- %s seconds has been take to move from (%f, %f) to (%f, %f)" % (time.time() - start_time, start[0], start[1], goal[0], goal[1]))
        has_reached = True
        return

    else:
        # 로봇이 가야할 방향 (회전해야할 방향 계산)
        tmp = math.atan2(dy, dx)#) - state[2]
        if tmp <= 0.05:
            tmp += 6.2832
        new_ang = tmp - state[2]
        if new_ang < -3.1415:
            new_ang += 6.2832
        elif new_ang > 3.1415:
            new_ang -= 6.2832
    
    # 충돌 확률이 ㅁ% 밑이면, 로봇을 목표점 방향으로 회전하도록 한다. (전역 경로 추종) 
    if collision_prob <= 0.1:
        twist_data.linear.x = beta * 0.26 + (1-beta) * twist_data.linear.x    
        twist_data.angular.z =  0.4 * new_ang * beta + (1-beta) * twist_data.angular.z

    # 충돌 확률이 높으면, 깊이 이미지 기반 회피 주행을 한다. (지역 경로 추종)    
    else:
        # 직진 속도 계산
        new_vel = (1 + control_out[1]) * c - collision_prob[0]*0.4
        twist_data.linear.x = beta * new_vel + (1-beta) * twist_data.linear.x

        # 좌우회전 확률 낮으면 그냥 직진
        if control_out[0] < 0.1 and control_out[2] < 0.1:
            twist_data.angular.z = 0
        
        # 우회전이랑 좌회전 확률 중 높은것만 채택
        # 좌회전인경우
        elif control_out[0] >= control_out[2]:
            # Angular velocity는 양수
            twist_data.angular.z = (0.1+ collision_prob[0] + control_out[0]) * beta * 0.7 + (1-beta) * twist_data.angular.z
        # 좌회전인 경우는 반대. 
        elif control_out[0] < control_out[2]:# and new_ang < 0:
            twist_data.angular.z = -(0.01 + collision_prob[0] + control_out[2]) * beta * 0.7 + (1-beta) * twist_data.angular.z

    # 충돌 확률이 높아 후진해야할 때, 후진 속도 제한
    if twist_data.linear.x < -0.1:
        twist_data.linear.x = -0.05

    pub.publish(twist_data)

    # 충돌 확률 : Matlab simulation을 위해 publish함.
    cols = Float32()
    cols.data = collision_prob[0]
    pub2.publish(cols)
    



def main():
    # global twist_data
    image_topic = "/camera/depth/image_raw"
    odom_topic = "/odom"

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
