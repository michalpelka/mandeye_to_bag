#include <vector>
#include <map>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include "LasLoader.h"

#include <livox_ros_driver/CustomMsg.h>
#include <livox_ros_driver/CustomPoint.h>

#include "common/ImuLoader.h"

#include <filesystem>


const double time_start = 0;
const int num_point = 19968;
const int line_count = 4;


int main(int argc, char **argv) {
    // parse command line arguments for directory to process
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " <directory> <output_bag>" << std::endl;
        std::cout << " Options are:" << std::endl;
        std::cout << "  --lines <number of lines> (default 8)" << std::endl;
        return 1;
    }
    int number_of_lines = 8; // default for mid360
    const std::string directory = argv[1];
    const std::string output_bag = argv[2];
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg.size() > 1 && arg[0] == '-' && arg[1] == '-') {
            if (arg == "--lines") {
                number_of_lines = std::stoi(argv[i + 1]);
                i++;
            } else {
                std::cout << "Unknown option: " << arg << std::endl;
                return 1;
            }
        }
    }

    std::cout << "Processing directory: " << directory << " creating bag " << output_bag << std::endl;
    std::cout << "Number of lines: " << number_of_lines << std::endl;
// get list of files in directory
    std::vector<std::string> files_imu;
    std::vector<std::string> files_laz;
    for (const auto &entry: std::filesystem::directory_iterator(directory)) {
        if (entry.path().extension() == ".csv") {
            files_imu.push_back(entry.path());
        } else if (entry.path().extension() == ".laz") {
            files_laz.push_back(entry.path());
        }
    }

    std::sort(files_imu.begin(), files_imu.end());
    std::sort(files_laz.begin(), files_laz.end());

    rosbag::Bag bag;
    bag.open(output_bag, rosbag::bagmode::Write);

    for (const auto& imu_fn : files_imu)
    {
        const auto data = mandeye::load_imu(imu_fn, 0);
        for (const auto& [ts, ang, acc] : data)
        {
            if (ts == 0)
            continue;
            sensor_msgs::Imu imu;
            imu.header.frame_id = "livox";
            imu.header.stamp.fromSec(time_start+ts);
            imu.angular_velocity.x = ang[0];
            imu.angular_velocity.y = ang[1];
            imu.angular_velocity.z = ang[2];

            imu.linear_acceleration.x = acc[0];
            imu.linear_acceleration.y = acc[1];
            imu.linear_acceleration.z = acc[2];

            bag.write("/livox/imu", imu.header.stamp, imu);
        }
    }

    livox_ros_driver::CustomMsg custom_msg;
    custom_msg.lidar_id = 192;
    custom_msg.header.frame_id = "livox_frame";
    custom_msg.rsvd[0] = 0;
    custom_msg.rsvd[1] = 0;
    custom_msg.rsvd[2] = 0;
    custom_msg.points.reserve(num_point);

    int line_id = 0;
    for (auto &pcName: files_laz) {
        auto points = mandeye::load(pcName);
        for (auto p: points) {
            if (p.timestamp == 0)
                continue;
            p.timestamp += time_start;
            if (custom_msg.points.size() == 0) {
                custom_msg.header.stamp.fromSec(p.timestamp);
                custom_msg.timebase = p.timestamp * 1e9;
            }
            livox_ros_driver::CustomPoint cp;
            cp.tag = 0;
            cp.offset_time = p.timestamp * 1e9 - custom_msg.timebase;
            cp.reflectivity = p.intensity;
            cp.x = p.point.x();
            cp.y = p.point.y();
            cp.z = p.point.z();
            cp.line = line_id;
            custom_msg.points.push_back(cp);
            if (line_id++ >= line_count) {
                line_id = 0;
                if (custom_msg.points.size() > num_point) {
                    custom_msg.point_num = custom_msg.points.size();
                    bag.write("/livox/lidar", custom_msg.header.stamp, custom_msg);
                    custom_msg.points.clear();
                }
            }
        }
    }
    bag.close();
    return 0;
}
