#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int16.h"
#include <stdlib.h>
#include <iostream>
// #include <windows.h>
// #include <chrono>
// #include <thread>

ros::Publisher new_pub;

void callback_f (const std_msgs::Int16 ct_msg) {
    geometry_msgs::Twist tw_msg;
    std::cout << ct_msg << std::endl;
    if(ct_msg.data == 0) {
        tw_msg.linear.x = 0;
        tw_msg.angular.z = 2;
    }
    else if(ct_msg.data == 1){
        tw_msg.linear.x = 0.1;
        tw_msg.angular.z = -2;
    }
    else if(ct_msg.data == 2){
        tw_msg.linear.x = 0.1;
        tw_msg.angular.z = 0;
    }
    else{
        tw_msg.linear.x = 0.1;
        tw_msg.angular.z = 2;
    }

    new_pub.publish(tw_msg);
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtlesim_depth_move");
    ros::NodeHandle nh;
    new_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Subscriber sub = nh.subscribe("ctr_out", 10, &callback_f);
    ros::spin();
    
    return 0;
}

