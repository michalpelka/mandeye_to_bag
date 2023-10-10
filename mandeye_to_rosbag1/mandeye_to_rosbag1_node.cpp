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

#include <filesystem>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

const double time_start = 1000;
const int num_point = 19968;
const int line_count = 4;


sensor_msgs::PointCloud2 createPointcloud(std::vector<mandeye::Point> &points)
{
    sensor_msgs::PointCloud2 points_msg;
    points_msg.header.stamp.fromSec(points.front().timestamp);
    points_msg.header.frame_id="livox";
    points_msg.height = 1; // if this is a "full 3D" pointcloud, height is 1; if this is Kinect-like pointcloud, height is the vertical resolution
    points_msg.width  = points.size();
    points_msg.is_bigendian = false;
    points_msg.is_dense = false; // there may be invalid points

    sensor_msgs::PointCloud2Modifier pcd_modifier(points_msg);
// this call also resizes the data structure according to the given width, height and fields
    pcd_modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
                         "y", 1, sensor_msgs::PointField::FLOAT32,
                         "z", 1, sensor_msgs::PointField::FLOAT32,
                         "i", 1, sensor_msgs::PointField::FLOAT32);

    sensor_msgs::PointCloud2Iterator<float> iter_x(points_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(points_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(points_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_i(points_msg, "i");

    for (auto pit = points.begin(); iter_x != iter_x.end(); ++pit, ++iter_x, ++iter_y, ++iter_z, ++iter_i)
    {
        // TODO fill in x, y, z, r, g, b local variables
        *iter_x = pit->point.x();
        *iter_y = pit->point.y();
        *iter_z = pit->point.z();
        *iter_i = pit->intensity;
    }

    return points_msg;
}
/// Tool converts a mandeye dataset to a rosbag with Livox messages `livox_ros_driver::CustomMsg`
int main(int argc, char **argv) {
    // parse command line arguments for directory to process
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " <directory> <output_bag>" << std::endl;
        std::cout << " Options are:" << std::endl;
        std::cout << "  --lines <number of lines> (default 8)" << std::endl;
        std::cout << "  --output <type of output : custom_msg / pointclous> (default pointcloud)" << std::endl;


        return 1;
    }
    int number_of_lines = 8; // default for mid360
    std::string type = "pointcloud";
    const std::string directory = argv[1];
    const std::string output_bag = argv[2];
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg.size() > 1 && arg[0] == '-' && arg[1] == '-') {
            if (arg == "--lines") {
                number_of_lines = std::stoi(argv[i + 1]);
                i++;
            }
            else if (arg=="--output")
            {
                type = argv[i+1];
                i++;
            }

            else {
                std::cout << "Unknown option: " << arg << std::endl;
                return 1;
            }
        }
    }

    std::cout << "Processing directory: " << directory << " creating bag " << output_bag << std::endl;
    std::cout << "Number of lines: " << number_of_lines << std::endl;
    std::cout << "Type of output: " << type << std::endl;

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

    for (const auto &imu_fn: files_imu) {
        std::ifstream myfile(imu_fn);
        if (myfile.is_open()) { // always check whether the file is open
            //myfile >> mystring; // pipe file's content into stream
            //std::cout << mystring; // pipe stream's content to standard output
            while (myfile) {
                double data[7];
                myfile >> data[0] >> data[1] >> data[2] >> data[3] >> data[4] >> data[5] >> data[6];
                //std::cout << data[0] << " " << data[1] << " " << data[2] << " " << data[3] << " " << data[4] << " " << data[5] << " " << data[6] << std::endl;
                if (data[0] > 0) {
                    sensor_msgs::Imu imu;
                    imu.header.frame_id = "livox";
                    imu.header.stamp.fromSec(time_start + data[0] / 1e9);
                    imu.angular_velocity.x = data[1];
                    imu.angular_velocity.y = data[2];
                    imu.angular_velocity.z = data[3];

                    imu.linear_acceleration.x = 0.1*data[4];
                    imu.linear_acceleration.y = 0.1*data[5];
                    imu.linear_acceleration.z = 0.1*data[6];

                    bag.write("/livox/imu", imu.header.stamp, imu);
                }
            }
            myfile.close();
        }
    }

    if (type == "custom_msg")
    {
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
    }
    else if (type=="pointcloud")
    {

        int line_id = 0;
        std::vector<mandeye::Point> chunk;
        double startTime = -1;
        for (auto &pcName: files_laz) {
            auto points = mandeye::load(pcName);

            for (auto p: points) {
                if (p.timestamp == 0 ){
                    continue;
                }
                if (startTime < 0)
                {
                    startTime = p.timestamp;
                }

                chunk.push_back(p);
                double diff = p.timestamp - startTime;
                if (diff > 0.1)
                {
                    auto msg = createPointcloud(chunk);
                    ros::Time t;
                    t.fromSec(startTime);
                    bag.write("/livox/lidar", t, msg);
                    std::cout << "wrote pointcloud";
                    chunk.clear();
                    startTime = p.timestamp;
                }
            }
        }

    }

    bag.close();
    return 0;
}