#!/usr/bin/env bash
set -euo pipefail
container=rc-ros-driver-sim
repo="$HOME/ai-engine-hackathon-2026"
auth_file="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}"
[[ $(docker inspect --format '{{.State.Running}} {{.HostConfig.NetworkMode}}' "$container" 2>/dev/null) == 'true none' ]] || {
    echo 'Start the mock driver first: bash ~/run-rc-ros-sim.sh' >&2
    exit 1
}
[[ -r "$auth_file" && -S /tmp/.X11-unix/X0 ]] || {
    echo 'Open a terminal on the Spark desktop and run this command there.' >&2
    exit 1
}
if docker exec "$container" pgrep -x rviz2 >/dev/null 2>&1; then
    echo 'RViz is already open on the Spark desktop.'
    exit 0
fi
mkdir -p "$repo/recording"
# A filesystem socket link exposes X11 without giving the mock driver host networking.
if [[ ! /tmp/.X11-unix/X0 -ef "$repo/recording/rviz-X0" ]]; then
    ln -f /tmp/.X11-unix/X0 "$repo/recording/rviz-X0"
fi
docker exec "$container" bash -lc '
mkdir -p /tmp/.X11-unix
ln -sfn /home/rosdev/ros2_ws/recording/rviz-X0 /tmp/.X11-unix/X0
'
docker exec -i "$container" bash -lc 'umask 077; cat > /tmp/rviz.Xauthority' < "$auth_file"
exec docker exec \
    -e DISPLAY=:0 -e XAUTHORITY=/tmp/rviz.Xauthority \
    -e QT_X11_NO_MITSHM=1 -e LIBGL_ALWAYS_SOFTWARE=1 \
    -e ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT \
    -e FASTDDS_DEFAULT_PROFILES_FILE=/home/rosdev/ros2_ws/tools/mock-fastdds.xml \
    "$container" bash -lc '
source /opt/ros/kilted/setup.bash
source /home/rosdev/robotiq_ws/install/local_setup.sh
source /home/rosdev/orbbec_ws/install/local_setup.sh
exec rviz2 -d /home/rosdev/ros2_ws/robot/mock-view.rviz
'
