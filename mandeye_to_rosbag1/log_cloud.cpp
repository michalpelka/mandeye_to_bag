#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <fstream>
#include <iostream>
std::ofstream csv_file;

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{

    if (!csv_file.is_open()) {
        ROS_ERROR("Failed to open the CSV file");
        return;
    }

    // Create iterators to iterate over the PointCloud2 data
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");

    // Iterate over the point cloud and write each point to the CSV file
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        csv_file << *iter_x << " " << *iter_y << " " << *iter_z << "\n";
    }

    csv_file.flush();

    ROS_INFO("PointCloud data has been written to the CSV file.");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_to_csv");
    ros::NodeHandle nh;

    // Open the CSV file

    csv_file.open("/tmp/output.csv");
    ros::Subscriber sub = nh.subscribe("/cloud_registered", 1, pointCloudCallback);

    ros::spin();

    return 0;
}
