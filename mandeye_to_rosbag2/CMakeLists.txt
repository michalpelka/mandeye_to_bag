cmake_minimum_required(VERSION 3.8)
project(mandeye_to_rosbag2)

find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set (CMAKE_CXX_STANDARD 17)

# LASzip library
add_subdirectory(common/3rd/LASzip)
include_directories(common/3rd/LASzip/include)

include_directories(common/)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

find_package(std_srvs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(rosbag2_to_mandeye_node rosbag2_to_mandeye_node.cpp common/LasLoader.cpp)
target_include_directories(rosbag2_to_mandeye_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(rosbag2_to_mandeye_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(rosbag2_to_mandeye_node rclcpp sensor_msgs rosbag2_cpp)
target_link_libraries(rosbag2_to_mandeye_node laszip Eigen3::Eigen)
install(TARGETS rosbag2_to_mandeye_node DESTINATION lib/${PROJECT_NAME})

add_executable(mandeye_to_rosbag2_node mandeye_to_rosbag2_node.cpp common/LasLoader.cpp common/ImuLoader.cpp)
target_include_directories(mandeye_to_rosbag2_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(mandeye_to_rosbag2_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(mandeye_to_rosbag2_node rclcpp sensor_msgs rosbag2_cpp)
target_link_libraries(mandeye_to_rosbag2_node laszip Eigen3::Eigen)
install(TARGETS mandeye_to_rosbag2_node DESTINATION lib/${PROJECT_NAME})


add_executable(log_trajectory log_trajectory.cpp)
ament_target_dependencies(log_trajectory rclcpp sensor_msgs nav_msgs)
target_include_directories(log_trajectory PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(log_trajectory PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS log_trajectory DESTINATION lib/${PROJECT_NAME})


add_executable(log_cloud log_cloud.cpp)
ament_target_dependencies(log_cloud rclcpp sensor_msgs)
target_include_directories(log_cloud PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(log_cloud PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS log_cloud DESTINATION lib/${PROJECT_NAME})

ament_package()

else()
  message("ament_cmake not found so skipping this package")
endif()