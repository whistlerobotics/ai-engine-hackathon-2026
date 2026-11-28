#!/usr/bin/env bash
set -eo pipefail

mock_state_only=false
if [[ ${1:-} == --mock-state-only ]]; then
    mock_state_only=true
    shift
fi

if [[ $# -gt 1 || ${1:-} == --help ]]; then
    echo "Usage: bash tools/record-three-cameras.sh [--mock-state-only] [new-output-directory]"
    echo "Run inside the ROS devcontainer. Ctrl-C saves and closes the recording."
    exit 0
fi

if $mock_state_only && [[ ${ROS_DOMAIN_ID:-} != 77 ]]; then
    echo "Mock state capture requires ROS_DOMAIN_ID=77." >&2
    exit 2
fi

source /opt/ros/kilted/setup.bash
set -u
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
output=${1:-recording/three-camera-$(date -u +%Y%m%dT%H%M%SZ)}
mkdir -p -- "$(dirname -- "$output")"

if $mock_state_only; then
    echo "Recording mock robot and controller state to $output"
    # The mock runner requires exactly one subscriber on each command topic.
    exec ros2 bag record --storage mcap --output "$output" --topics \
        /joint_states \
        /gripper/joint_states \
        /joint_trajectory_controller/controller_state
fi

echo "Recording the existing camera topics and robot state to $output"
exec ros2 bag record --storage mcap --output "$output" \
    --qos-profile-overrides-path "$script_dir/three-camera-recording-qos.yaml" \
    --topics \
    /left_camera/color/image_raw/compressed \
    /right_camera/color/image_raw/compressed \
    /wrist_camera/color/image_raw/compressed \
    /left_camera/color/camera_info \
    /right_camera/color/camera_info \
    /wrist_camera/color/camera_info \
    /joint_states \
    /gripper/joint_states \
    /joint_trajectory_controller/joint_trajectory \
    /joint_trajectory_controller/controller_state \
    /gripper/finger_position_controller/commands \
    /hand_guiding/active \
    /speed_scaling_state_broadcaster/speed_scaling \
    /io_and_status_controller/robot_program_running \
    /io_and_status_controller/robot_mode \
    /io_and_status_controller/safety_mode \
    /tcp_pose_broadcaster/pose \
    /tf /tf_static
