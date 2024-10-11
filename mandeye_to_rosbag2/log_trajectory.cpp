#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <iostream>
#include <fstream>

std::ofstream csv_file;

void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Log the trajectory points to the console

    const auto& position = msg->pose.pose.position;
    const auto& orientation = msg->pose.pose.orientation;

    csv_file << (uint64_t) (msg->header.stamp.sec*1e9 + msg->header.stamp.nanosec) <<"," ;
    csv_file << position.x << "," << position.y << "," << position.z << ",";
    csv_file << orientation.x << "," << orientation.y << "," << orientation.z << "," << orientation.w<< std::endl;

}

int main(int argc, char** argv)
{

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("trajectory_logger_node");

    std::string topic_name = node->declare_parameter<std::string>("topic", "/Odometry");
    std::string output_file_name = node->declare_parameter<std::string>("output_file", "output.csv");

    csv_file.open(output_file_name);
    auto sub = node->create_subscription<nav_msgs::msg::Odometry>(topic_name, 10, odometryCallback);

    rclcpp::spin(node);
    rclcpp::shutdown();
    csv_file.close();
    return 0;
}
