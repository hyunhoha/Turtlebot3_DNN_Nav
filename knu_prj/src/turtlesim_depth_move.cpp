#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int16.h"
#include <stdlib.h>
#include <iostream>

ros::Publisher *pub;

void control_msg_received(const std_msgs::Int16 ct_msg){
    geometry_msgs::Twist tw_msg;
    if(ct_msg.data == 0) {
        tw_msg.linear.x = 0;
        tw_msg.angular.z = 10;
    }
    else if(ct_msg.data == 1){
        tw_msg.linear.x = 5;
        tw_msg.angular.z = -10;
    }
    else if(ct_msg.data == 2){
        tw_msg.linear.x = 10;
        tw_msg.angular.z = 0;
    }
    else{
        tw_msg.linear.x = 5;
        tw_msg.angular.z = 10;
    }
    std::cout << tw_msg.linear.x << " " << tw_msg.angular.z << std::endl;
    pub->publish(tw_msg);
}

int main(int argc, char **argv)
{
    // Ros initialization
    ros::init(argc, argv, "turtlesim_depth_move");
    ros::NodeHandle nh;
    // geometry_msgs::Twist tw_msg;

    // Create a publisher object
    pub = new ros::Publisher(nh.advertise<geometry_msgs::Twist>("cmd_vel",1));
    ros::Subscriber sub = nh.subscribe("ctr_out", 1, &control_msg_received);
    
    
    ros::spin();
    // srand(time(0));
    delete pub;
}

//     ros::Rate rate(2);
//     while(ros::ok())
//     {
//         // Create and fill in the msgs, ...
//         geometry_msgs::Twist msg;
//         msg.linear.x = double(rand())*2. / double(RAND_MAX) - 1.;
//         // msg.linear.y = double(rand()) / double(RAND_MAX) - 1.;
//         msg.angular.z = 2.*double(rand()) / double(RAND_MAX) -1.;
//         // msg.linear.x = 0.3;
//         // msg.linear.y = 0.3;
//         // msg.linear.y = 0;
//         // msg.angular.z = 0.3;
//         // msg.linear.z = 0;
//         // msg.angular.x = 0;
//         // msg.angular.y = 0;
//         // msg.angular.x = 2.*double(rand());

//         pub.publish(msg);
//         // pub2.publish(msg);

//         ROS_INFO_STREAM("Sending random velocity command:"
//                         << " linear_x=" << msg.linear.x
//                         // << " linear_y=" << msg.linear.y
//                         << " angular=" << msg.angular.z);

//         rate.sleep();
//     }
//     return 0;
// }