#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <fstream>

void odometryCallback(const nav_msgs::Odometry& msg)
{
    // Log the trajectory points to the file

        const auto& position = msg.pose.pose.position;
        const auto& orientation = msg.pose.pose.orientation;

        std::cout << position.x << " " << position.y << " " << position.z << " ";
        std::cout << orientation.x << " " << orientation.y << " " << orientation.z << " " << orientation.w << " ";
        std::cout << msg.header.stamp.toSec() <<std::endl;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_logger_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/Odometry", 10, odometryCallback);

    ros::spin();


    return 0;
}

