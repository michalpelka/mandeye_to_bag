#pragma once
#include <vector>
#include <Eigen/Dense>
#include <laszip/laszip_api.h>
namespace mandeye
{
    struct Point {
        double timestamp;
        float intensity;
        Eigen::Vector3d point;
    };

    std::vector<Point> load(const std::string& lazFile );
    bool saveLaz(const std::string& filename, const std::vector<mandeye::Point>& buffer);
}