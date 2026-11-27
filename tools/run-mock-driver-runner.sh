#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/kilted/setup.bash
source /home/rosdev/robotiq_ws/install/local_setup.sh
set -u
export ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT
export FASTDDS_DEFAULT_PROFILES_FILE="$(dirname -- "${BASH_SOURCE[0]}")/mock-fastdds.xml"
[[ ${ROS_DOMAIN_ID:-} == 77 ]] || { echo "Mock runner requires ROS domain 77" >&2; exit 1; }
task=${1:-mock-driver-check}
policy_url=${2:-http://127.0.0.1:18000}
printf -v runner_command '%q ' \
    /home/rosdev/ros2_ws/target/release/policy-runner \
    --task "$task" --policy "$policy_url" \
    --camera-topic /right_camera/color/image_raw/compressed --camera-size 640x360
exec script -q -f -e --command "$runner_command" /tmp/ros-driver-ui.log
