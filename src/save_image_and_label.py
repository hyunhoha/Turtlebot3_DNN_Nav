#! /usr/bin/python3

# rospy for the subscriber
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from nav_msgs.msg import Odometry

# Instantiate CvBridge
bridge = CvBridge()

# Velocity가 들어올 때 마다 global 변수인 현재 속도를 갱신한다.
def velocity_callback(msg):
    global current_vel
    current_vel = msg
    if msg.linear.x >= 0.25:
        current_vel.linear.x = 0.25
    else:
        current_vel.linear.x = round(msg.linear.x, 2)

    current_vel.angular.z = round(msg.angular.z, 2)
            
# 이미지가 들어올 때 현재 속도랑 이미지를 동시에 저장한다.
def image_callback(msg):
    # print("Received an image!")
    # print(count)
    global img_count
    global count
    global current_vel
    # print(img_count)

    try:
        # 이미지 들어오는 속도가 너무 빠르면 조절할 수 있음.
        # img_count라는걸 새로 만들어서, 속도를 n배 늦출 수 있음.
        # count = 이미지 저장용 숫자.

        if img_count % 5 == 0:   # 늦추려면 5를 더 크게 변경

            if current_vel.linear.x < 0 or (current_vel.linear.x == 0 and current_vel.angular.z == 0):
                return None

            cv2_img = bridge.imgmsg_to_cv2(msg, "32FC1")#"bgr8")
            cv_image_array = np.array(cv2_img, dtype = np.dtype('f8'))
            cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 255, cv2.NORM_MINMAX)

            # 이미지 저장 폴더 경로, txt파일 경로             
            s='/home/hyun/catkin_ws/src/knu_prj/ㅁㅁ/camera_image'+str(count)+'.jpeg'
            s2 = '/home/hyun/catkin_ws/src/knu_prj/ㅁㅁ.txt'
            
            cv2.imwrite(s, cv_image_norm)
            f = open(s2, 'a')
            f.write(str(current_vel.linear.x) + " " + str(current_vel.angular.z) + "\n")

            count += 1
            img_count=1
            
        else:
            img_count += 1

    except:
        print("Except")

def main():
    global img_count
    global count
    

    rospy.init_node('image_listener_save')
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