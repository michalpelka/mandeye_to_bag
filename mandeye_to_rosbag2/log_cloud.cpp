#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

std::ofstream csv_file;
std::string output_file_name;
void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
    csv_file.open(output_file_name);

    if (!csv_file.is_open())
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to open the CSV file");
        return;
    }

    // Create iterators to iterate over the PointCloud2 data
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");

    // Iterate over the point cloud and write each point to the CSV file
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
        csv_file << *iter_x << " " << *iter_y << " " << *iter_z << "\n";
    }

    csv_file.flush();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PointCloud data has been written to the CSV file.");
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("pointcloud_to_csv");

    std::string topic_name = node->declare_parameter<std::string>("topic", "/Odometry");
    output_file_name = node->declare_parameter<std::string>("output_file", "output.csv");

    // Create the subscription
    auto sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(topic_name, 1, pointCloudCallback);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
