#!/usr/bin/env bash
set -eo pipefail
# ros-env imports every message package in inherited overlays, including unrelated camera bindings.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH
source /opt/ros/kilted/setup.bash
source /home/rosdev/robotiq_ws/install/local_setup.sh
set -u

repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$repo_dir"
exec cargo run --locked --release -p record -- "$@"
