#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/kilted/setup.bash
source /home/rosdev/robotiq_ws/install/local_setup.sh
set -u

repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
task=${1:-bolt_to_tray}
if [[ $# -gt 0 ]]; then shift; fi

exec "$repo_dir/target/release/policy-runner" \
    --task "$task" \
    --policy http://127.0.0.1:18081 \
    --camera-topic /right_camera/color/image_raw/compressed \
    --camera-size 640x360 \
    "$@"
