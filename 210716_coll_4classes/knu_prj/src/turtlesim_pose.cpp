#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include <iomanip>
#include "nav_msgs/Odometry.h"
#include <math.h>

// void poseMessageReceived(const turtlesim::Pose &msg) {
void poseMessageReceived(const nav_msgs::Odometry &msg) {
    // float pos_x = msg.x;
    // float pos_y = msg.y;
    float direction = atan((2*msg.pose.pose.orientation.w * msg.pose.pose.orientation.z)/(1-2*pow(msg.pose.pose.orientation.z,2)));
    ROS_INFO_STREAM(std::setprecision(2) << std::fixed <<
        "position = (" << msg.pose.pose.position.x << "," << msg.pose.pose.position.y<< ")" <<
        "direction = " << direction);
}

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "turtlesim_pose");
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe("odom", 1000, &poseMessageReceived);

    ros::spin();

    return 0;
}