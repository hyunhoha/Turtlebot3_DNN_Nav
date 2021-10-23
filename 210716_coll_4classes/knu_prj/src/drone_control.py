#! /usr/bin/python3
import collections
import rospy
# ROS Image message
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError

# from mavros_msgs.msg import *
from geometry_msgs.msg import TwistStamped  
# OpenCV2 for saving an image
# from gazebo_msgs import ModelStates

import cv2
import numpy as np
import tensorflow as tf
from sklearn.preprocessing import normalize

from tensorflow.compat.v1 import ConfigProto
from tensorflow.compat.v1 import InteractiveSession

from nav_msgs.msg import Odometry
import tf as trf

config = ConfigProto()
config.gpu_options.allow_growth = True
session = InteractiveSession(config=config)


bridge = CvBridge()
model = tf.keras.models.load_model("/home/hyun/catkin_ws/src/knu_prj/res_collision_model_new_4_classes_0_labeled/")
control_out = 0
pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

linx = 0
# liny = 0
angz = 0
collision_prob = 0

vel_max = 0.3


def image_callback(image_data):
    # global control_out
    global twist_data
    global linx
    global angz
    global collision_prob

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

    a, b, c, d = -0.05, 3, 0.2, 3
    
    alpha = 0.9
    # new_vel = (control_out[0] * a) + (control_out[2] * c) - collision_prob[0]*0.15
    new_vel = (1 - control_out[0] + control_out[2]) * c - collision_prob[0]*0.1
    # twist_local.twist.linear.x = alpha * new_vel + (1-alpha) * twist_local.twist.linear.x 
    linx = alpha * new_vel + (1-alpha) * linx
    # if collision_prob[0] >= 0.1:
        
    # else:
    #     twist_data.linear.x = alpha * new_vel + (1-alpha) * twist_data.linear.x

    if control_out[1] >= control_out[3]:
        angz = (0.01+ control_out[0] + collision_prob[0]) * alpha * 0.5 + (1-alpha) * angz
    else:
        angz = -(0.01 + control_out[0] + collision_prob[0]) * alpha * 0.5 + (1-alpha) * angz

    # pub.publish(twist_data)
    # print("control : ", twist_data.twist.linear.x, twist_data.twist.angular.z)
    # print("control : ", )
    # return linx, angz

def pos_callback(odom_data):
    kp = 1
    kd = 2
    kt = 1
    ktd = 2
    
    rot = odom_data.pose.pose.orientation
    p = odom_data.pose.pose.position
    curz = trf.transformations.euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])[2]
    twist_data.twist.linear.z = kp * (posz - p.z)
    if collision_prob < 0.3:
        twist_data.twist.linear.x = kp * (posx-p.x)
        twist_data.twist.linear.y = kp * (posy-p.y)
        twist_data.twist.angular.z = -kt * (np.pi/2 - np.arctan2(posy-p.y, posx-p.x) + curz)
    else:
        twist_data.twist.linear.x = kp * (posx-p.x) + kd * linx
        twist_data.twist.linear.y = kp * (posy-p.y)
        twist_data.twist.angular.z = -kt * (np.pi/2 - np.arctan2(posy-p.y, posx-p.x) + curz) + ktd * angz
    
    # if abs(twist_data.twist.linear.x) >= vel_max:
    #     if twist_data.twist.linear.x < 0:
    #         twist_data.twist.linear.x = -vel_max
    #     else:
    #         twist_data.twist.linear.x = vel_max
    
    # if abs(twist_data.twist.linear.y) >= vel_max:
    #     if twist_data.twist.linear.y < 0:
    #         twist_data.twist.linear.y = -vel_max
    #     else:
    #         twist_data.twist.linear.y = vel_max

    pub.publish(twist_data)
    print("ctrl cmd published! : ", twist_data.twist.linear.x, "  " , twist_data.twist.linear.y, " ",  twist_data.twist.linear.z, " " , twist_data.twist.angular.z)


def main():
    # global twist_data
    image_topic = "/camera/depth/image_raw"

    
    # status_topic = "/gazebo/model_states/"
    
    # rospy.Subscriber(status_topic, ModelStates, status_callback)
    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.Subscriber('/mavros/local_position/odom',Odometry, pos_callback)
    rospy.spin()

if __name__ == '__main__':
    control_out = 0
    rospy.init_node('image_listener')
    rate = rospy.Rate(10)
    twist_data = TwistStamped()
    twist_data.twist.linear.y = 0
    twist_data.twist.linear.z = 0
    twist_data.twist.angular.x = 0
    twist_data.twist.angular.y = 0

    posx = 5
    posy = 7
    posz = 4

    # crush_state = False
    while not rospy.is_shutdown():
        main()
        rate.sleep()
