#!/usr/bin/env bash
set -euo pipefail
container=rc-ros-driver-sim
if ! docker container inspect "$container" >/dev/null 2>&1; then
    docker run -d --name "$container" --network none --user 1000:1000 \
        --cap-drop ALL --security-opt no-new-privileges \
        -e ROS_DOMAIN_ID=77 -e ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT \
        -e FASTDDS_DEFAULT_PROFILES_FILE=/home/rosdev/ros2_ws/tools/mock-fastdds.xml \
        -e ROS_HOME=/tmp/ros \
        -v "$HOME/ai-engine-hackathon-2026:/home/rosdev/ros2_ws:ro" \
        -w /home/rosdev/ros2_ws/robot --entrypoint bash whistle-hackathon:local -lc '
source /opt/ros/kilted/setup.bash
source /home/rosdev/robotiq_ws/install/local_setup.sh
source /home/rosdev/orbbec_ws/install/local_setup.sh
exec ros2 launch launch.xml robot:=mock robot_ip:=127.0.0.1 \
    camera:=false hand_guiding:=false headless_mode:=false
' >/dev/null
else
    [[ $(docker inspect --format '{{.HostConfig.NetworkMode}}' "$container") == none ]] || {
        echo "Refusing to use a simulation container with external networking" >&2
        exit 1
    }
    docker start "$container" >/dev/null
fi

if ! docker exec "$container" python3 -c '
import json, urllib.request
with urllib.request.urlopen("http://127.0.0.1:18000/readyz", timeout=2) as r:
    assert json.load(r)["ready"]
' >/dev/null 2>&1; then
    docker exec -d "$container" bash -lc '
exec python3 /home/rosdev/ros2_ws/test-policy.py --port 18000 \
    --amplitude 0.02 --joints 5 --delay 0 --log-requests \
    > /tmp/mock-policy.log 2>&1
'
fi

docker exec -d relaxed_clarke bash -lc '
source /opt/ros/kilted/setup.bash
exec python3 /home/rosdev/ros2_ws/tools/bridge-real-cameras.py export \
    >> /tmp/mock-camera-export.log 2>&1
'
docker exec -d "$container" bash -lc '
source /opt/ros/kilted/setup.bash
exec python3 /home/rosdev/ros2_ws/tools/bridge-real-cameras.py import \
    >> /tmp/mock-camera-import.log 2>&1
'
docker exec "$container" bash -lc '
source /opt/ros/kilted/setup.bash
python3 /home/rosdev/ros2_ws/tools/bridge-real-cameras.py check
'

[[ ${1:-} != --prepare-only ]] || exit 0

echo 'Mock ROS driver: isolated domain 77, networking disabled, no physical devices.'
echo 'Uses a deterministic wrist/gripper test policy with real camera frames.'
echo 'Left, right and wrist feeds are copied into the mock domain; the runner monitors right.'
echo 'First Enter moves to start; second Enter runs. Esc aborts and holds; q quits while idle.'
exec docker exec -it "$container" \
    bash /home/rosdev/ros2_ws/tools/run-mock-driver-runner.sh
