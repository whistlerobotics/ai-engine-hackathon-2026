#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/kilted/setup.bash
source /home/rosdev/robotiq_ws/install/local_setup.sh
set -u
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
[[ ${ROS_DOMAIN_ID:-} == 77 ]] || { echo "Mock runner requires ROS domain 77" >&2; exit 1; }
printf -v runner_command '%q ' \
    /home/rosdev/ros2_ws/target/release/policy-runner \
    --task mock-driver-check --policy http://127.0.0.1:18000 \
    --camera-topic /right_camera/color/image_raw/compressed --camera-size 640x360
exec script -q -f -e --command "$runner_command" /tmp/ros-driver-ui.log
