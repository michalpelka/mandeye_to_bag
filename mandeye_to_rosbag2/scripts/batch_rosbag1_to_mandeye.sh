#!/usr/bin/env bash
set -euo pipefail

script_name=$(basename "$0")

usage() {
    cat <<EOF
Usage: ${script_name} <input_dir_with_ros1_bags> <output_base_dir> [rosbag2_to_mandeye_node options...]

Runs rosbag1_to_mandeye.sh on every *.bag file found in <input_dir_with_ros1_bags>
(non-recursive), writing each converted mandeye dataset to:
  <output_base_dir>/<bag_basename>/

Any extra arguments are forwarded to rosbag2_to_mandeye_node, e.g.:
  ${script_name} /data/bags /data/mandeye --lidar_type ouster
EOF
}

if [[ $# -lt 2 ]]; then
    usage
    exit 1
fi

input_dir="$1"
output_base_dir="$2"
shift 2

if [[ "${output_base_dir}" == -* ]]; then
    echo "Error: missing <output_base_dir> argument (got '${output_base_dir}', which looks like an option)." >&2
    usage
    exit 1
fi

if [[ ! -d "${input_dir}" ]]; then
    echo "Error: input directory not found: ${input_dir}" >&2
    exit 1
fi

if ! command -v ros2 >/dev/null 2>&1; then
    echo "Error: ros2 command not found. Source your ROS 2 workspace first." >&2
    exit 1
fi

shopt -s nullglob
bags=("${input_dir}"/*.bag)
shopt -u nullglob

if [[ ${#bags[@]} -eq 0 ]]; then
    echo "No .bag files found in ${input_dir}" >&2
    exit 1
fi

mkdir -p "${output_base_dir}"

failed_bags=()
for bag in "${bags[@]}"; do
    bag_name=$(basename "${bag}" .bag)
    out_dir="${output_base_dir}/${bag_name}"
    echo "=== Processing ${bag} -> ${out_dir} ==="
    if ! ros2 run mandeye_to_rosbag2 rosbag1_to_mandeye.sh "${bag}" "${out_dir}" "$@"; then
        echo "Warning: failed to process ${bag}" >&2
        failed_bags+=("${bag}")
    fi
done

if [[ ${#failed_bags[@]} -gt 0 ]]; then
    echo "Done with errors. Failed bags:" >&2
    printf '  %s\n' "${failed_bags[@]}" >&2
    exit 1
fi

echo "All bags processed successfully."