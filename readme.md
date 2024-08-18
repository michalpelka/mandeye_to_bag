# Tools to Convert Mandeye Data to ROSBag

This repository contains a simple tool that enables the conversion of Mandeye data to ROSBag.

[![ROS 1 Build](https://github.com/michalpelka/mandeye_to_bag/actions/workflows/ros1-build.yml/badge.svg)](https://github.com/michalpelka/mandeye_to_bag/actions/workflows/ros1-build.yml)
[![ROS 2 Build](https://github.com/michalpelka/mandeye_to_bag/actions/workflows/ros2-build.yml/badge.svg)](https://github.com/michalpelka/mandeye_to_bag/actions/workflows/ros2-build.yml)

# ROS 1
## Building 

To build the tool, follow these steps:

1. Create a ROS1 workspace:
```
mkdir -p ~/mandeye_ws/src
```

2. Clone the repository:
```
cd ~/mandeye_ws/src
git clone https://github.com/michalpelka/mandeye_to_bag.git
```

3. Initialize third-party repositories:
```
cd ~/mandeye_ws/src/mandeye_to_bag
vcs import --input mandeye_to_bag1/livox.repos 
```

4. Initilize sumbodules
```
cd ~/mandeye_ws/src/mandeye_to_bag/mandeye_to_rosbag1
git submodule update --init --recursive
```
5. Install dependencies
```
cd ~/mandeye_ws/src/mandeye_to_bag
rosdep update && rosdep install --from-paths src --ignore-src -r -y
```
6. Build the workspace:
```
cd ~/mandeye_ws
catkin_make
```

## Usage

To convert Mandeye data, run the following command:
```
rosrun mandeye_to_rosbag1 mandeye_to_rosbag /media/michal/F318-56A3/continousScanning_0000 /home/michal/testPalka.bag
```
Make sure to replace `/media/michal/F318-56A3/continousScanning_0000` with the path to your Mandeye data and `/home/michal/testPalka.bag` with the path to your desired output ROSBag file.

## Using the Tool with FAST-LIO

To use the tool with FAST-LIO, follow these steps:

1. Launch FAST-LIO in the first terminal (make sure that your workspace is sourced):
```
roslaunch fast_lio mapping_avia.launch 
```

2. Play the converted ROSBag in the second terminal:
```
rosbag play /home/michal/testPalka.bag
```

Note: Replace `/home/michal/testPalka.bag` with the path to your converted ROSBag file.

