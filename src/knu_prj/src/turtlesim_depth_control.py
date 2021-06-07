#! /home/iasl/anaconda3/bin/python3
########! /usr/bin/python3
########! /home/iasl/anaconda3/bin/python3

import rospy
import sys
# ROS Image message
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
# ROS Image message -> OpenCV2 image converter

sys.path.remove('/home/iasl/catkin_ws/opt/ros/melodic/lib/python2.7/dist-packages')
sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')

# import cv_bridge.boost.cv_bridge_boost
import cv_bridge.boost
from cv_bridge import CvBridge, CvBridgeError

# OpenCV2 for saving an image


import numpy as np
import cv2

# sys.path.append('/usr/local/lib/python3.7/site-packages')

import tensorflow as tf
from sklearn.preprocessing import normalize
# from PIL import Imagep
# import scipy.misc
# import matplotlib
print("\n".join(sys.path))



bridge = CvBridge()
model = tf.keras.models.load_model("/home/iasl/catkin_ws/src/knu_prj/src/DNN_model/")
# img = cv2.imread("/home/iasl/catkin_ws/src/knu_prj/src/camera_image32.jpeg", cv2.IMREAD_ANYCOLOR)
# img_re = cv2.resize(img, dsize=(640,480), interpolation=cv2.INTER_LINEAR)
# image = np.array(img_re).reshape((1,640,480,1))

def image_callback(msg):
    global control_out
    # try:
    # im = np.frombuffer(image_data.data, dtype=np.uint32).reshape(image_data.height, image_data.width)
    # cv2.imwrite("/home/iasl/catkin_ws/src/knu_prj/test.png", im)
    # print(im)
    # image = Image.fromarray(im)
    # scipy.misc.toimage("/home/iasl/catkin_ws/src/knu_prj/test2.png", im)
    # matplotlib.image.imsave('/home/iasl/catkin_ws/src/knu_prj/test2.png',im)
    # image.save("/home/iasl/catkin_ws/src/knu_prj/test2.png")
    # print(im.shape)
    
    #####
    print("0!")
    # cv2_img = bridge.imgmsg_to_cv2(msg,0) #"32FC1")
    cv2_img = bridge.imgmsg_to_cv2(msg,"32FC1") #"32FC1")
    print("1!")
    cv_image_array = np.array(cv2_img, dtype = np.dtype('f8'))
    print("2!")
    cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 255, cv2.NORM_MINMAX)
    # np.linalg.norm
    # cv_image_norm = normalize(im)
    # cv_image_norm = cv2.normalize(im, None, 0, 255, cv2.NORM_MINMAX)
    print("3!")
    img_re = cv2.resize(cv_image_norm, dsize=(640,480), interpolation=cv2.INTER_LINEAR)
    print("4!")
    image = np.array(img_re).reshape((1,640,480,1))
    print("5!")
    control_out = np.argmax(model.predict(image))
    ######
    # except:
    #     print("Exception!")

def main():

    image_topic = "/camera/depth/image_raw"
    rospy.Subscriber(image_topic, Image, image_callback)
    pub = rospy.Publisher('ctr_out', Int16 , queue_size =10)
    pub.publish(control_out)
    print("control : ", control_out)
    # msg = Twist.cmd_vel
    # if control_out == 0:
    #     msg.linear_x = 0
    #     msg.angular_z = 10
    rospy.spin()

if __name__ == '__main__':
    control_out = 0
    rospy.init_node('image_listener')
    main()

# int main(int argc, char **argv)
# {
#     // Ros initialization
#     ros::init(argc, argv, "turtlesim_random_move");
#     ros::NodeHandle nh;

#     // Create a publisher object
#     ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);
#     // ros::Publisher pub2 = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1000);

#     // send the random #
#     srand(time(0));

#     ros::Rate rate(2);
#     while(ros::ok())
#     {
#         // Create and fill in the msgs, ...
#         geometry_msgs::Twist msg;
#         msg.linear.x = double(rand())*2. / double(RAND_MAX) - 1.;
#         // msg.linear.y = double(rand()) / double(RAND_MAX) - 1.;
#         msg.angular.z = 2.*double(rand()) / double(RAND_MAX) -1.;
#         // msg.linear.x = 0.3;
#         // msg.linear.y = 0.3;
#         // msg.linear.y = 0;
#         // msg.angular.z = 0.3;
#         // msg.linear.z = 0;
#         // msg.angular.x = 0;
#         // msg.angular.y = 0;
#         // msg.angular.x = 2.*double(rand());

#         pub.publish(msg);
#         // pub2.publish(msg);

#         ROS_INFO_STREAM("Sending random velocity command:"
#                         << " linear_x=" << msg.linear.x
#                         // << " linear_y=" << msg.linear.y
#                         << " angular=" << msg.angular.z);

#         rate.sleep();
#     }
#     return 0;
# }