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
#include <algorithm>
#include <numeric>
#include <filesystem>
#include <livox_ros_driver/CustomMsg.h>
#include <livox_ros_driver/CustomPoint.h>
//! \brief GetInterpolatedTimstampForLidarPoint
//! \param frameRate - frame rate of the lidar in seconds
//! \param startTs - timestamp of the first point in the point cloud
//! \param numPoints - number of points in the point cloud
//! \param pointNumber - number of the point to get the timestamp for
//! \return interpolated timestamp for the point in seconds
double GetInterpolatedTimstampForLidarPoint(const double frameRate, const double startTs, const int numPoints, const int pointNumber)
{
    return startTs + (pointNumber * frameRate) / numPoints;
}


void SaveData(const std::string& output_directory,const int count, const std::vector<mandeye::Point>& buffer_pointcloud, const std::vector<std::string>& buffer_imu)
{
    namespace fs = std::filesystem;
    fs::create_directory(output_directory);
    char fn[1024];
    sprintf(fn, "%s/pointcloud_%04d.laz", output_directory.c_str(), count);
    mandeye::saveLaz(fn, buffer_pointcloud);
    sprintf(fn, "%s/imu_%04d.csv",output_directory.c_str(), count);
    std::ofstream f(fn);
    for (const auto &imu : buffer_imu) {
        f << imu << std::endl;
    }
    f.close();
}
// Tool converts a rosbag with  sensor_msgs::PointCloud2 to a mandeye dataset
int main(int argc, char **argv) {
    // parse command line arguments for directory to process
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " <input_bag_directory> <directory>" << std::endl;
        std::cout << " Options are:" << std::endl;
        std::cout << "  --pointcloud_topic " << std::endl;
        std::cout << "  --imu_topic " << std::endl;
        std::cout << "  --chunk_len " << std::endl;
        std::cout << " --emulate_point_ts" << std::endl;

        return 1;
    }
    const std::string input_bag_directory = argv[1];
    const std::string output_directory = argv[2];

    bool emulate_point_ts = false;
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
            else if (arg == "--emulate_point_ts") {
                emulate_point_ts = true;
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
    std::cout << "Emulate point ts : " << emulate_point_ts << std::endl;
    std::vector<std::string> files_bag;

    if (input_bag_directory.find(".bag") != std::string::npos)
    {
        files_bag.push_back(input_bag_directory);
    }
    else{
        // get list of files in directory
        for (const auto &entry: std::filesystem::directory_iterator(input_bag_directory)) {
            if (entry.path().extension() == ".bag") {
                files_bag.push_back(entry.path());
            }
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
        double last_imu_timestamp = -1.;
        double lidarFrameRate = 0.0;

        // collect framerate of lidar
        if (emulate_point_ts)
        {
            std::cout << "Emulating point timestamps, collecting framerate for 20 sec of bag." << std::endl;
            std::vector<double> header_diffs;
            double last_header_ts = 0.0;
            double start_ts = 0.0;
            for (const rosbag::MessageInstance& msg : view) {
                if (msg.getTopic() == pointcloud_topic && msg.isType<sensor_msgs::PointCloud2>())
                {
                    sensor_msgs::PointCloud2::ConstPtr cloud_msg = msg.instantiate<sensor_msgs::PointCloud2>();
                    assert(cloud_msg != nullptr);
                    if (last_header_ts != 0.0)
                    {
                        start_ts = last_header_ts;
                        double diff = cloud_msg->header.stamp.toSec() - last_header_ts;
                        if (diff > 0.0)
                        {
                            header_diffs.push_back(diff);
                        }
                        if(cloud_msg->header.stamp.toSec() - start_ts > 20.0)
                        {
                            break;
                        }
                    }
                    last_header_ts = cloud_msg->header.stamp.toSec();

                }
            }

            const double average_diff = std::accumulate(header_diffs.begin(), header_diffs.end(), 0.0)/  header_diffs.size();
            const double variance = std::accumulate(header_diffs.begin(), header_diffs.end(), 0.0, [average_diff](double acc, double val) { return acc + (val - average_diff) * (val - average_diff); }) / header_diffs.size();
            std::cout << "Average header diff: " << average_diff << " std_dev : " << std::sqrt(variance) << std::endl;
            lidarFrameRate = average_diff;
        }

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
            if (msg.getTopic() == pointcloud_topic && msg.isType<livox_ros_driver::CustomMsg>() && last_imu_timestamp > 0.0)
            {
                livox_ros_driver::CustomMsg::ConstPtr custom_msg = msg.instantiate<livox_ros_driver::CustomMsg>();
                for (const auto& cp : custom_msg->points)
                {
                	const uint64_t ts = custom_msg->timebase + cp.offset_time;
                	mandeye::Point point;
                	point.point.x() = cp.x;
                    point.point.y() = cp.y;
                    point.point.z() = cp.z;
                    point.intensity = cp.reflectivity;
                    point.timestamp = ts;
                    buffer_pointcloud.push_back(point);
                }
               
            }
            
            if (msg.getTopic() == pointcloud_topic && msg.isType<sensor_msgs::PointCloud2>() && last_imu_timestamp > 0.0)
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
                    const double headerTimestampS = cloud_msg->header.stamp.toSec();
                    const int num_points = cloud_msg->width * cloud_msg->height;
                    int point_counter = 0;
                    for (; x_it != x_it.end(); ++x_it, ++y_it, ++z_it, ++i_it) {
                        mandeye::Point point;
                        point.point.x() = *x_it;
                        point.point.y() = *y_it;
                        point.point.z() = *z_it;
                        point.intensity = *i_it;
                        if (emulate_point_ts) {
                            point.timestamp = GetInterpolatedTimstampForLidarPoint(lidarFrameRate, headerTimestampS, num_points, point_counter) * 1e9;

                        } else {
                            point.timestamp = cloud_msg->header.stamp.toNSec();
                        }
                        buffer_pointcloud.push_back(point);
                        point_counter++;
                    }
                }else{
                    double error = std::abs(ts-last_imu_timestamp);
                    std::cout << "Skipping pointcloud: " << cloud_msg->header.stamp <<" difference to imu " << error << std::endl;
                }
            }


            if ( msg.getTime().toSec() - last_save_timestamp > chunk_len && last_save_timestamp > 0.0) {
                SaveData(output_directory, count,buffer_pointcloud, buffer_imu);
                buffer_pointcloud.clear();
                buffer_imu.clear();
                last_save_timestamp = msg.getTime().toSec();
                count++;
            }
        }
        if (buffer_pointcloud.size() > 0) {
            SaveData(output_directory, count, buffer_pointcloud, buffer_imu);
        }

    }

    return 0;
}
