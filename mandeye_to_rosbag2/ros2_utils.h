#pragma once
#include <common.h>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <iostream>
double GetSecondFromRosTime(const builtin_interfaces::msg::Time& time)
{
    return static_cast<double>(time.sec) + static_cast<double>(time.nanosec) / 1e9;
}

std::uint64_t GetNanoFromRosTime(const builtin_interfaces::msg::Time& time)
{
    return static_cast<std::uint64_t>(time.sec) * 1e9 + static_cast<std::uint64_t>(time.nanosec);
}

builtin_interfaces::msg::Time GetRosTimeSecond(double time)
{
    assert(time >= 0.0); // time should be positive
    assert(time < std::numeric_limits<int32_t>::max()); // time should be less than max of int32_t
    builtin_interfaces::msg::Time ros_time;
    ros_time.sec = static_cast<int32_t>(time);
    double fractional_seconds = time - ros_time.sec;
    ros_time.nanosec = static_cast<int32_t>(fractional_seconds * 1e9);
    return ros_time;
}


builtin_interfaces::msg::Time GetRosTimeFromNanoSecond(double timeNano)
{
    const double time = timeNano / 1e9;
    return GetRosTimeSecond(time);
}

sensor_msgs::msg::PointCloud2 CreatePointcloudMessage(const std::vector<mandeye::Point> points)
{
    sensor_msgs::msg::PointCloud2 pc2;
    pc2.header.frame_id = "livox";
    pc2.height = 1;
    pc2.width = points.size();
    sensor_msgs::PointCloud2Modifier mod(pc2);
    mod.setPointCloud2Fields(
        4,
        "x",
        1,
        sensor_msgs::msg::PointField::FLOAT32,
        "y",
        1,
        sensor_msgs::msg::PointField::FLOAT32,
        "z",
        1,
        sensor_msgs::msg::PointField::FLOAT32,
        "intensity",
        1,
        sensor_msgs::msg::PointField::FLOAT32);
    sensor_msgs::PointCloud2Iterator<float> iter_x(pc2, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(pc2, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(pc2, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_i(pc2, "intensity");

    for (auto p : points)
    {
        *iter_x = p.point.x();
        *iter_y = p.point.y();
        *iter_z = p.point.z();
        *iter_i = p.intensity;
        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_i;
    }
    return pc2;
}
