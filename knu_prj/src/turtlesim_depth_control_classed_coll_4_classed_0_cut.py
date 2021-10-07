#! /usr/bin/python3
import rospy
# ROS Image message
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
# from std_msgs.msg import Int16
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
# from gazebo_msgs import ModelStates

import cv2
import numpy as np
import tensorflow as tf
from sklearn.preprocessing import normalize

from tensorflow.compat.v1 import ConfigProto
from tensorflow.compat.v1 import InteractiveSession

config = ConfigProto()
config.gpu_options.allow_growth = True
session = InteractiveSession(config=config)


bridge = CvBridge()
# model = tf.keras.models.load_model("/home/iasl/catkin_ws/src/knu_prj/src/DNN_model/")
# model = tf.keras.models.load_model("/home/iasl/catkin_ws/src/knu_prj/res_class_model2/")
# model = tf.keras.models.load_model("/home/iasl/catkin_ws/src/knu_prj/res_collision_model_new_4_classes_0_labeled/")

model = tf.keras.models.load_model("/home/iasl/catkin_ws/src/knu_prj/res_collision_model_new_4_classes_0_labeled_double/")
# model = tf.keras.models.load_model("/home/iasl/catkin_ws/src/knu_prj/new_res_model/")
# model = tf.keras.models.load_model("/home/iasl/catkin_ws/src/knu_prj/res_collision_model_new_4_classes_0_labeled_small/")
control_out = 0
# pub = rospy.Publisher('ctr_out', Int16 , queue_size =10)
pub = rospy.Publisher('cmd_vel', Twist , queue_size =10)

def image_callback(image_data):
    # global control_out
    global twist_data
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

    a, b, c, d = -0.05, 1, 0.2, 3
    alpha = 0.9
    # new_vel = (control_out[0] * a) + (control_out[2] * c) - collision_prob[0]*0.15
    new_vel = (1 - control_out[0] + control_out[2]) * c - collision_prob[0]*0.1
    # new_vel = control_out[2] * c - collision_prob[0] * 0.05
    twist_data.linear.x = alpha * new_vel + (1-alpha) * twist_data.linear.x 
    # if collision_prob[0] >= 0.1:
        
    # else:
    #     twist_data.linear.x = alpha * new_vel + (1-alpha) * twist_data.linear.x
    # print("control : " , control_out, collision_prob[0])
    if control_out[1] >= control_out[3]:
        twist_data.angular.z = (0.01+ control_out[0] + collision_prob[0]) * b #* alpha * 0.5 + (1-alpha) * twist_data.angular.z
        # twist_data.angular.z = (1-alpha) * twist_data.angular.z + alpha  *  (1+ collision_prob[0]*5) * b
    else:
        twist_data.angular.z = -(0.01 + control_out[0] + collision_prob[0]) * b# * alpha * 0.5 + (1-alpha) * twist_data.angular.z
        # twist_data.angular.z = (1-alpha) * twist_data.angular.z - alpha  *  (1+ collision_prob[0]*5) * b

    pub.publish(twist_data)

def main():
    image_topic = "/camera/depth/image_raw"
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
