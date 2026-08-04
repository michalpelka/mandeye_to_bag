#include "LasLoader.h"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/reader.hpp"
#include <algorithm>
#include <filesystem>
#include <iostream>
#include <map>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
//#include <rosbag2_cpp/storage_options.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <vector>
#include "ros2_utils.h"

void SaveData(
    const std::string& output_directory,
    const int count,
    const std::vector<mandeye::Point>& buffer_pointcloud,
    const std::vector<std::string>& buffer_imu)
{
    namespace fs = std::filesystem;
    fs::create_directory(output_directory);
    char fn[1024];
    sprintf(fn, "%s/pointcloud_%04d.laz", output_directory.c_str(), count);
    mandeye::saveLaz(fn, buffer_pointcloud);
    sprintf(fn, "%s/imu_%04d.csv", output_directory.c_str(), count);
    std::ofstream f(fn);
    for (const auto& imu : buffer_imu)
    {
        f << imu << std::endl;
    }
    f.close();
}

enum class LidarType
{
    Ouster,
    Hesai,
    Livox
};

LidarType LidarTypeFromString(const std::string& str)
{
    if (str == "ouster")
    {
        return LidarType::Ouster;
    }
    else if (str == "hesai")
    {
        return LidarType::Hesai;
    }
    else if (str == "livox")
    {
        return LidarType::Livox;
    }
    throw std::runtime_error("Unknown lidar type: " + str + ", supported types are: ouster, hesai, livox");
}

std::string LidarTypeToString(const LidarType type)
{
    switch (type)
    {
    case LidarType::Ouster:
        return "ouster";
    case LidarType::Hesai:
        return "hesai";
    case LidarType::Livox:
        return "livox";
    }
    return "unknown";
}

std::string DefaultPointcloudTopicForLidar(const LidarType type)
{
    switch (type)
    {
    case LidarType::Ouster:
        return "/ouster/points";
    case LidarType::Hesai:
        return "/hesai/pandar";
    case LidarType::Livox:
        return "/livox/lidar";
    }
    return "";
}

std::string DefaultImuTopicForLidar(const LidarType type)
{
    switch (type)
    {
    case LidarType::Ouster:
        return "/ouster/imu";
    case LidarType::Hesai:
        return ""; // hesai lidars typically do not publish an IMU topic
    case LidarType::Livox:
        return "/livox/imu";
    }
    return "";
}

// Tool converts a rosbag with  sensor_msgs::PointCloud2 to a mandeye dataset
int main(int argc, char** argv)
{
    // parse command line arguments for directory to process
    if (argc < 3)
    {
        std::cout << "Usage: " << argv[0] << " <input_bag> <directory>" << std::endl;
        std::cout << " Options are:" << std::endl;
        std::cout << "  --pointcloud_topic <topic> (default depends on --lidar_type)" << std::endl;
        std::cout << "  --imu_topic <topic>        (default depends on --lidar_type)" << std::endl;
        std::cout << "  --chunk_len " << std::endl;
        std::cout << "  --lidar_type <ouster|hesai|livox>  (default: ouster)" << std::endl;

        return 1;
    }
    const std::string input_bag = argv[1];
    const std::string output_directory = argv[2];

    std::string pointcloud_topic;
    std::string imu_topic;
    bool pointcloud_topic_set = false;
    bool imu_topic_set = false;
    float chunk_len = 20.0f;
    LidarType lidar_type = LidarType::Ouster;
    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];
        if (arg.size() > 1 && arg[0] == '-' && arg[1] == '-')
        {
            if (arg == "--pointcloud_topic")
            {
                pointcloud_topic = argv[i + 1];
                pointcloud_topic_set = true;
                i++;
            }
            else if (arg == "--imu_topic")
            {
                imu_topic = argv[i + 1];
                imu_topic_set = true;
                i++;
            }
            else if (arg == "--chunk_len")
            {
                chunk_len = std::stof(argv[i + 1]);
                i++;
            }
            else if (arg == "--lidar_type")
            {
                lidar_type = LidarTypeFromString(argv[i + 1]);
                i++;
            }
            else
            {
                std::cout << "Unknown option: " << arg << std::endl;
                return 1;
            }
        }
    }

    // fall back to per-lidar default topics unless explicitly overridden
    if (!pointcloud_topic_set)
    {
        pointcloud_topic = DefaultPointcloudTopicForLidar(lidar_type);
    }
    if (!imu_topic_set)
    {
        imu_topic = DefaultImuTopicForLidar(lidar_type);
    }

    std::cout << "Processing directory of bags: " << pointcloud_topic << " creating mandeye dataset " << output_directory << std::endl;
    std::cout << "Pointcloud topic : " << pointcloud_topic << std::endl;
    std::cout << "Imu topic        : " << imu_topic << std::endl;
    std::cout << "Chunk len        : " << chunk_len << std::endl;
    std::cout << "Lidar type       : " << LidarTypeToString(lidar_type) << std::endl;

    std::vector<mandeye::Point> buffer_pointcloud;
    std::vector<std::string> buffer_imu;
    double last_save_timestamp = 0.0;
    unsigned int count = 0;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializationPointCloud2;
    rclcpp::Serialization<sensor_msgs::msg::Imu> serializationImu;


    std::cout << "Processing bag: " << input_bag << std::endl;
    rosbag2_cpp::Reader bag;

    bag.open(input_bag);

    double last_imu_timestamp = -1.;

    while(bag.has_next())
    {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = bag.read_next();
        if (msg->topic_name== imu_topic)
        {
            std::shared_ptr<sensor_msgs::msg::Imu> imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            serializationImu.deserialize_message(&serialized_msg, imu_msg.get());
            assert(imu_msg != nullptr);
            std::stringstream ss;
            ss << GetNanoFromRosTime(imu_msg->header.stamp) << " " << imu_msg->angular_velocity.x << " " << imu_msg->angular_velocity.y << " "
               << imu_msg->angular_velocity.z << " " << imu_msg->linear_acceleration.x << " " << imu_msg->linear_acceleration.y << " "
               << imu_msg->linear_acceleration.z;
            buffer_imu.push_back(ss.str());
            last_imu_timestamp = GetSecondFromRosTime(imu_msg->header.stamp);
            if (last_save_timestamp == 0.0)
            {
                last_save_timestamp = GetSecondFromRosTime(imu_msg->header.stamp);
            }
        }
        if (msg->topic_name == pointcloud_topic && last_imu_timestamp > 0.0)
        {

            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            std::shared_ptr<sensor_msgs::msg::PointCloud2> cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            serializationPointCloud2.deserialize_message(&serialized_msg, cloud_msg.get());
            assert(cloud_msg != nullptr);

            double ts = GetSecondFromRosTime(cloud_msg->header.stamp);
            assert(cloud_msg != nullptr);
            // Create point cloud iterators
            sensor_msgs::PointCloud2ConstIterator<float> x_it(*cloud_msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> y_it(*cloud_msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> z_it(*cloud_msg, "z");
            sensor_msgs::PointCloud2ConstIterator<float> i_it(*cloud_msg, "intensity");

            if (std::abs(ts - last_imu_timestamp) < 0.05 * chunk_len)
            {
                const double headerTimestampS = GetSecondFromRosTime(cloud_msg->header.stamp);

                if (lidar_type == LidarType::Ouster)
                {
                    // ouster: field "t" is a per-point uint32 nanosecond offset from the header stamp
                    sensor_msgs::PointCloud2ConstIterator<uint32_t> ts_it(*cloud_msg, "t");
                    for (; x_it != x_it.end(); ++x_it, ++y_it, ++z_it, ++i_it, ++ts_it)
                    {
                        mandeye::Point point;
                        point.point.x() = *x_it;
                        point.point.y() = *y_it;
                        point.point.z() = *z_it;
                        point.intensity = *i_it;
                        point.timestamp = headerTimestampS * 1e9 + double(*ts_it);
                        buffer_pointcloud.push_back(point);
                    }
                }
                else // LidarType::Hesai or LidarType::Livox
                {
                    // hesai/livox: field "timestamp" is a per-point float64 absolute unix time in seconds
                    sensor_msgs::PointCloud2ConstIterator<double> ts_it(*cloud_msg, "timestamp");
                    for (; x_it != x_it.end(); ++x_it, ++y_it, ++z_it, ++i_it, ++ts_it)
                    {
                        mandeye::Point point;
                        point.point.x() = *x_it;
                        point.point.y() = *y_it;
                        point.point.z() = *z_it;
                        point.intensity = *i_it;
                        point.timestamp = *ts_it * 1e9;
                        buffer_pointcloud.push_back(point);
                    }
                }
            }
            else
            {
                double error = std::abs(ts - last_imu_timestamp);
                std::cout << "Skipping pointcloud: " << GetSecondFromRosTime(cloud_msg->header.stamp) << " difference to imu " << error << std::endl;
            }
        }

        const double messageTimeInSeconds = static_cast<double>(msg->recv_timestamp)/1e9;
        if (messageTimeInSeconds - last_save_timestamp > chunk_len && last_save_timestamp > 0.0)
        {
            SaveData(output_directory, count, buffer_pointcloud, buffer_imu);
            buffer_pointcloud.clear();
            buffer_imu.clear();
            last_save_timestamp = messageTimeInSeconds;
            count++;
        }
    }
    if (buffer_pointcloud.size() > 0)
    {
        SaveData(output_directory, count, buffer_pointcloud, buffer_imu);
    }

    return 0;
}
