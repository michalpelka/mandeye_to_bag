#!/usr/bin/env bash
set -euo pipefail

script_name=$(basename "$0")

usage() {
    cat <<EOF
Usage: ${script_name} <input_ros1_bag> <output_mandeye_dir> [rosbag2_to_mandeye_node options...]

Converts a ROS 1 bag into a mandeye dataset:
  1. Converts <input_ros1_bag> to a temporary ROS 2 bag (via rosbags-convert)
  2. Runs rosbag2_to_mandeye_node on that ROS 2 bag to produce <output_mandeye_dir>
  3. Deletes the temporary ROS 2 bag

Any extra arguments are forwarded to rosbag2_to_mandeye_node, e.g.:
  ${script_name} input.bag /path/to/dataset --pointcloud_topic /ouster/points --lidar_type ouster
EOF
}

if [[ $# -lt 2 ]]; then
    usage
    exit 1
fi

input_ros1_bag="$1"
output_mandeye_dir="$2"
shift 2

if [[ "${output_mandeye_dir}" == -* ]]; then
    echo "Error: missing <output_mandeye_dir> argument (got '${output_mandeye_dir}', which looks like an option)." >&2
    usage
    exit 1
fi

if [[ ! -f "${input_ros1_bag}" ]]; then
    echo "Error: input ROS 1 bag not found: ${input_ros1_bag}" >&2
    exit 1
fi

if ! command -v rosbags-convert >/dev/null 2>&1; then
    echo "Error: rosbags-convert not found. Install it with: pip install rosbags" >&2
    exit 1
fi

tmp_ros2_bag=$(mktemp -u -d -t mandeye_ros2_bag_XXXXXX)

cleanup() {
    rm -rf "${tmp_ros2_bag}"
}
trap cleanup EXIT

echo "Converting ROS 1 bag '${input_ros1_bag}' to ROS 2 bag '${tmp_ros2_bag}'..."
rosbags-convert --src "${input_ros1_bag}" --dst "${tmp_ros2_bag}"

echo "Converting ROS 2 bag '${tmp_ros2_bag}' to mandeye dataset '${output_mandeye_dir}'..."
ros2 run mandeye_to_rosbag2 rosbag2_to_mandeye_node "${tmp_ros2_bag}" "${output_mandeye_dir}" "$@"

echo "Done. Mandeye dataset written to '${output_mandeye_dir}'."