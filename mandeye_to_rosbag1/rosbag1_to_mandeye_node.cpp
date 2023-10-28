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
        std::cout << "  --chunk_len " << std::endl;


        return 1;
    }
    const std::string input_bag_directory = argv[1];
    const std::string output_directory = argv[2];

    std::string pointcloud_topic = "/livox/lidar";
    std::string imu_topic = "";
    float chunk_len = 20.0f;
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
            }
            else if (arg == "--chunk_len") {
                chunk_len = std::stof(argv[i + 1]);
                i++;
            }
            else {
                std::cout << "Unknown option: " << arg << std::endl;
                return 1;
            }
        }
    }

    std::cout << "Processing directory of bags: " << pointcloud_topic << " creating mandeye dataset " << output_directory << std::endl;
    std::cout << "Pointcloud topic : " << pointcloud_topic << std::endl;
    std::cout << "Imu topic        : " << imu_topic << std::endl;
    std::cout << "Chunk len        : " << chunk_len << std::endl;

// get list of files in directory
    std::vector<std::string> files_bag;
    for (const auto &entry: std::filesystem::directory_iterator(input_bag_directory)) {
        if (entry.path().extension() == ".bag") {
            files_bag.push_back(entry.path());
        }
    }
    std::sort(files_bag.begin(), files_bag.end());


    std::vector<mandeye::Point> buffer_pointcloud;
    std::vector<std::string> buffer_imu;
    double last_save_timestamp = 0.0;
    unsigned int count = 0;
    for (auto & bag_file : files_bag)
    {
        std::cout << "Processing bag: " << bag_file << std::endl;
        rosbag::Bag bag;
        bag.open(bag_file, rosbag::bagmode::Read);
        std::vector<std::string> topics;
        topics.push_back(pointcloud_topic);
        topics.push_back(imu_topic);
        rosbag::View view(bag);
        double last_imu_timestamp = 0.0;

        for (const rosbag::MessageInstance& msg : view) {


            if (msg.getTopic() == imu_topic && msg.isType<sensor_msgs::Imu>())
            {
                sensor_msgs::Imu::ConstPtr imu_msg = msg.instantiate<sensor_msgs::Imu>();
                assert(imu_msg != nullptr);
                std::stringstream ss;
                ss << imu_msg->header.stamp.toNSec() << " " <<  imu_msg->angular_velocity.x << " " << imu_msg->angular_velocity.y
                << " " << imu_msg->angular_velocity.z << " "
                << imu_msg->linear_acceleration.x << " " << imu_msg->linear_acceleration.y << " "
                << imu_msg->linear_acceleration.z;
                buffer_imu.push_back(ss.str());
                last_imu_timestamp = imu_msg->header.stamp.toSec() ;
                if (last_save_timestamp == 0.0) {
                    last_save_timestamp = imu_msg->header.stamp.toSec();
                }
            }
            if (msg.getTopic() == pointcloud_topic && msg.isType<sensor_msgs::PointCloud2>())
            {
                sensor_msgs::PointCloud2::ConstPtr cloud_msg = msg.instantiate<sensor_msgs::PointCloud2>();
                double ts = cloud_msg->header.stamp.toSec();
                assert(cloud_msg != nullptr);
                // Create point cloud iterators
                sensor_msgs::PointCloud2ConstIterator<float> x_it(*cloud_msg, "x");
                sensor_msgs::PointCloud2ConstIterator<float> y_it(*cloud_msg, "y");
                sensor_msgs::PointCloud2ConstIterator<float> z_it(*cloud_msg, "z");
                sensor_msgs::PointCloud2ConstIterator<float> i_it(*cloud_msg, "intensity");


                if (std::abs(ts-last_imu_timestamp) < 0.05*chunk_len) {
                    for (; x_it != x_it.end(); ++x_it, ++y_it, ++z_it, ++i_it) {
                        mandeye::Point point;
                        point.point.x() = *x_it;
                        point.point.y() = *y_it;
                        point.point.z() = *z_it;
                        point.intensity = *i_it;
                        point.timestamp = cloud_msg->header.stamp.toNSec();
                        buffer_pointcloud.push_back(point);
                    }
                }else{
                    double error = std::abs(ts-last_imu_timestamp);
                    std::cout << "Skipping pointcloud: " << cloud_msg->header.stamp <<" difference to imu " << error << std::endl;
                }
            }


            if (  msg.getTime().toSec() - last_save_timestamp > chunk_len) {
                char fn[1024];
                sprintf(fn, "%s/pointcloud_%04d.laz", output_directory.c_str(), count);
                mandeye::saveLaz(fn, buffer_pointcloud);
                sprintf(fn, "%s/imu_%04d.csv",output_directory.c_str(), count);
                std::ofstream f(fn);
                for (const auto &imu : buffer_imu) {
                    f << imu << std::endl;
                }
                f.close();
                buffer_pointcloud.clear();
                buffer_imu.clear();
                last_save_timestamp = msg.getTime().toSec();
                count++;
            }


        }

    }

    return 0;
}