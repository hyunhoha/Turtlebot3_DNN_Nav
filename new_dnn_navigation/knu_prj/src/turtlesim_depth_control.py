#! /usr/bin/python3
import rospy
# ROS Image message
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image

import cv2
import numpy as np
import tensorflow as tf
from sklearn.preprocessing import normalize

bridge = CvBridge()
model = tf.keras.models.load_model("/home/iasl/catkin_ws/src/knu_prj/src/DNN_model/")
control_out = 0
pub = rospy.Publisher('ctr_out', Int16 , queue_size =10)

def image_callback(image_data):
    global control_out
    depth_image = bridge.imgmsg_to_cv2(image_data, "32FC1")
    cv_image_array = np.array(depth_image, dtype = np.dtype('f8'))
    img_re = cv2.resize(cv_image_array, dsize=(640,480), interpolation=cv2.INTER_LINEAR)
    img_norm = cv2.normalize(img_re, img_re, 0, 255, cv2.NORM_MINMAX)
    img_norm = np.uint8(img_norm)
    image = np.array(img_norm).reshape((1,480,640,1))
    control_out = np.argmax(model.predict(image))
    pub.publish(control_out)
    print("control : ", control_out)

def main():
    image_topic = "/camera/depth/image_raw"
    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.spin()

    
if __name__ == '__main__':
    control_out = 0
    rospy.init_node('image_listener')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        main()
        rate.sleep()
