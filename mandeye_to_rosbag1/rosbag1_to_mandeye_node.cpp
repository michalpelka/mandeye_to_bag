#include <vector>
#include <map>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include "LasLoader.h"
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <filesystem>


// Tool converts a rosbag with  sensor_msgs::PointCloud2 to a mandeye dataset
int main(int argc, char **argv) {
    // parse command line arguments for directory to process
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " <input_bag_directory> <directory>" << std::endl;
        std::cout << " Options are:" << std::endl;
        std::cout << "  --pointcloud_topic " << std::endl;
        std::cout << "  --imu_topic " << std::endl;

        return 1;
    }
    const std::string input_bag_directory = argv[1];
    const std::string output_directory = argv[2];

    std::string pointcloud_topic = "/livox/lidar";
    std::string imu_topic = "";
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg.size() > 1 && arg[0] == '-' && arg[1] == '-') {
            if (arg == "--pointcloud_topic") {
                pointcloud_topic = argv[i + 1];
                i++;
            }
            else if (arg == "--imu_topic") {
                imu_topic = argv[i + 1];
                i++;
            }else {
                std::cout << "Unknown option: " << arg << std::endl;
                return 1;
            }
        }
    }

    std::cout << "Processing directory of bags: " << pointcloud_topic << " creating mandeye dataset " << output_directory << std::endl;
    std::cout << "Pointclould topic: " << pointcloud_topic << std::endl;
    std::cout << "Imu topic: " << imu_topic << std::endl;

// get list of files in directory
    std::vector<std::string> files_bag;
    for (const auto &entry: std::filesystem::directory_iterator(input_bag_directory)) {
        if (entry.path().extension() == ".bag") {
            files_bag.push_back(entry.path());
        }
    }
    std::sort(files_bag.begin(), files_bag.end());

    for (auto & bag_file : files_bag)
    {
        std::cout << "Processing bag: " << bag_file << std::endl;
        rosbag::Bag bag;
        bag.open(bag_file, rosbag::bagmode::Read);
        std::vector<std::string> topics;
        topics.push_back(pointcloud_topic);
        topics.push_back(imu_topic);
        rosbag::View view(bag);
        for (const rosbag::MessageInstance& msg : view) {

            if (msg.getTopic() == pointcloud_topic && msg.isType<sensor_msgs::PointCloud2>())
            {
                sensor_msgs::PointCloud2::ConstPtr cloud_msg = msg.instantiate<sensor_msgs::PointCloud2>();
                assert(cloud_msg != nullptr);
                std::cout << "Processing pointcloud: " << cloud_msg->header.stamp << std::endl;

                // Create point cloud iterators
                sensor_msgs::PointCloud2ConstIterator<float> x_it(*cloud_msg, "x");
                sensor_msgs::PointCloud2ConstIterator<float> y_it(*cloud_msg, "y");
                sensor_msgs::PointCloud2ConstIterator<float> z_it(*cloud_msg, "z");
                for (; x_it != x_it.end(); ++x_it, ++y_it, ++z_it)
                {
                    float x = *x_it;
                    float y = *y_it;
                    float z = *z_it;

                    // Process x, y, z values
                    // Example: Print the point coordinates
                    std::cout << "Point: x=" << x << ", y=" << y << ", z=" << z << std::endl;
                }
            }
        }

    }

    return 0;
}