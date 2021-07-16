#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <stdlib.h>

int main(int argc, char **argv)
{
    // Ros initialization
    ros::init(argc, argv, "turtlesim_random_move");
    ros::NodeHandle nh;

    // Create a publisher object
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);
    // ros::Publisher pub2 = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1000);

    // send the random #
    srand(time(0));

    ros::Rate rate(2);
    while(ros::ok())
    {
        // Create and fill in the msgs, ...
        geometry_msgs::Twist msg;
        msg.linear.x = double(rand())*2. / double(RAND_MAX) - 1.;
        // msg.linear.y = double(rand()) / double(RAND_MAX) - 1.;
        msg.angular.z = 2.*double(rand()) / double(RAND_MAX) -1.;
        // msg.linear.x = 0.3;
        // msg.linear.y = 0.3;
        // msg.linear.y = 0;
        // msg.angular.z = 0.3;
        // msg.linear.z = 0;
        // msg.angular.x = 0;
        // msg.angular.y = 0;
        // msg.angular.x = 2.*double(rand());

        pub.publish(msg);
        // pub2.publish(msg);

        ROS_INFO_STREAM("Sending random velocity command:"
                        << " linear_x=" << msg.linear.x
                        // << " linear_y=" << msg.linear.y
                        << " angular=" << msg.angular.z);

        rate.sleep();
    }
    return 0;
}