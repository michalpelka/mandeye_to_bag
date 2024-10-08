#pragma once
#include <array>
#include <vector>
#include <string>
namespace mandeye
{
   using ImuAcceleration = std::array<float,3> ;
   using ImuAngularVelocity = std::array<float,3>;
   std::vector<std::tuple<double, ImuAngularVelocity, ImuAcceleration>> load_imu(const std::string& imu_file, int imuToUse);
}

