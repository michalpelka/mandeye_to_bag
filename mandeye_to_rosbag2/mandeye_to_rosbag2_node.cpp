#include <map>
#include <vector>

#include "LasLoader.h"
#include <sensor_msgs/msg/imu.hpp>

#include "LasLoader.h"
#include "rclcpp/serialization.hpp"
#include "ros2_utils.h"
#include "rosbag2_cpp/writer.hpp"
#include <algorithm>
#include <filesystem>
#include <iostream>
#include <map>
#include <numeric>
#include <optional>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/storage_options.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <vector>
#include "common/ImuLoader.h"
const int num_point = 19968;
const int line_count = 4;

int main(int argc, char** argv)
{
    // parse command line arguments for directory to process
    if (argc < 3)
    {
        std::cout << "Usage: " << argv[0] << " <directory> <output_bag>" << std::endl;
        std::cout << " Options are:" << std::endl;
        std::cout << "  --lines <number of lines> (default 8)" << std::endl;
        std::cout << "  --type <pointcloud2/livox> default is pointcloud2" << std::endl;
        return 1;
    }

    int number_of_lines = 8; // default for mid360
    std::string messageType = "pointcloud2";
    const std::string directory = argv[1];
    const std::string output_bag = argv[2];
    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];
        if (arg.size() > 1 && arg[0] == '-' && arg[1] == '-')
        {
            if (arg == "--lines")
            {
                number_of_lines = std::stoi(argv[i + 1]);
                i++;
            }
            else if (arg == "--type")
            {
                messageType = std::string(argv[i + 1]);
                i++;
            }
            else
            {
                std::cout << "Unknown option: " << arg << std::endl;
                return 1;
            }
        }
    }

    assert(messageType != "livox"); // "Only pointcloud2 is supported at this time.";

    std::cout << "Processing directory: " << directory << " creating bag " << output_bag << std::endl;
    std::cout << "Number of lines: " << number_of_lines << std::endl;

    // get list of files in directory
    std::vector<std::string> files_imu;
    std::vector<std::string> files_laz;
    for (const auto& entry : std::filesystem::directory_iterator(directory))
    {
        if (entry.path().extension() == ".csv")
        {
            files_imu.push_back(entry.path());
        }
        else if (entry.path().extension() == ".laz")
        {
            const std::string fn = entry.path().filename().string();
            if (fn.find("lidar") != std::string::npos)
            files_laz.push_back(entry.path());
        }
    }

    std::sort(files_imu.begin(), files_imu.end());
    std::sort(files_laz.begin(), files_laz.end());

    rosbag2_cpp::Writer bag;
    bag.open(output_bag);

    for (const auto& imu_fn : files_imu)
    {
        auto data = mandeye::load_imu(imu_fn, 0);
        for (const auto& [ts, ang, acc] : data)
        {
            if (ts == 0)
            continue;
            sensor_msgs::msg::Imu imu;
            imu.header.frame_id = "livox";
            imu.header.stamp = GetRosTimeSecond(ts);
            imu.angular_velocity.x = ang[0];
            imu.angular_velocity.y = ang[1];
            imu.angular_velocity.z = ang[2];

            imu.linear_acceleration.x = acc[0];
            imu.linear_acceleration.y = acc[1];
            imu.linear_acceleration.z = acc[2];

            bag.write(imu, "/livox/imu", imu.header.stamp);
        }
    }
    if (messageType == "pointcloud2")
    {
        std::vector<mandeye::Point> points;
        std::optional<double> last_ts;
        for (auto& pcName : files_laz)
        {
            auto new_points = mandeye::load(pcName);
            for (auto p : new_points)
            {
                if (p.timestamp == 0)
                    continue;
                if (!last_ts)
                {
                    last_ts = p.timestamp;
                }
                points.push_back(p);
                if (points.size() > num_point && last_ts && last_ts > 0)
                {
                    sensor_msgs::msg::PointCloud2 pc2 = CreatePointcloudMessage(points);
                    pc2.header.stamp = GetRosTimeSecond(*last_ts);
                    pc2.header.frame_id = "livox";
                    bag.write(pc2, "/livox/pointcloud", pc2.header.stamp);
                    points.clear();
                    last_ts = nullopt;
                }
            }
        }
    }

    //    livox_ros_driver::CustomMsg custom_msg;
    //    custom_msg.lidar_id = 192;
    //    custom_msg.header.frame_id = "livox_frame";
    //    custom_msg.rsvd[0] = 0;
    //    custom_msg.rsvd[1] = 0;
    //    custom_msg.rsvd[2] = 0;
    //    custom_msg.points.reserve(num_point);
    //
    //    int line_id = 0;
    //    for (auto &pcName: files_laz) {
    //        auto points = mandeye::load(pcName);
    //        for (auto p: points) {
    //            if (p.timestamp == 0)
    //                continue;
    //            p.timestamp += time_start;
    //            if (custom_msg.points.size() == 0) {
    //                custom_msg.header.stamp.fromSec(p.timestamp);
    //                custom_msg.timebase = p.timestamp * 1e9;
    //            }
    //            livox_ros_driver::CustomPoint cp;
    //            cp.tag = 0;
    //            cp.offset_time = p.timestamp * 1e9 - custom_msg.timebase;
    //            cp.reflectivity = p.intensity;
    //            cp.x = p.point.x();
    //            cp.y = p.point.y();
    //            cp.z = p.point.z();
    //            cp.line = line_id;
    //            custom_msg.points.push_back(cp);
    //            if (line_id++ >= line_count) {
    //                line_id = 0;
    //                if (custom_msg.points.size() > num_point) {
    //                    custom_msg.point_num = custom_msg.points.size();
    //                    bag.write("/livox/lidar", custom_msg.header.stamp, custom_msg);
    //                    custom_msg.points.clear();
    //                }
    //            }
    //        }
    //    }
    //
    bag.close();
    return 0;
}
