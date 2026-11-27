#!/usr/bin/env bash
set -euo pipefail
exec docker exec -it relaxed_clarke \
    bash /home/rosdev/ros2_ws/tools/run-three-camera-policy.sh "$@"
