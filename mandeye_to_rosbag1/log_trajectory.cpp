#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <fstream>

std::ofstream file;
void odometryCallback(const nav_msgs::Odometry& msg)
{
    // Log the trajectory points to the file

        const auto& position = msg.pose.pose.position;
        const auto& orientation = msg.pose.pose.orientation;
        file << (uint64_t) msg.header.stamp.toNSec() <<",";
        file << position.x << "," << position.y << "," << position.z << ",";
        file << orientation.x << "," << orientation.y << "," << orientation.z << "," << orientation.w << std::endl;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_logger_node");
    ros::NodeHandle nh;
    std::string file_path = "trajectory.txt";
    nh.getParam("file_path", file_path);
    std::cout << "File path: " << file_path << std::endl;
    file.open(file_path);
    
    std::cout << "Logging trajectory to file: " << file_path << std::endl;

    std::string topic_name = "Odometry";
    nh.getParam("topic_name", topic_name);
    std::cout << "Topic name: " << topic_name << std::endl;


    ros::Subscriber sub = nh.subscribe(topic_name, 10, odometryCallback);

    ros::spin();


    return 0;
}

