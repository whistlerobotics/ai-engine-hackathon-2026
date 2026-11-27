#!/usr/bin/env bash
set -euo pipefail
container=rc-ros-driver-sim
bash "$HOME/run-rc-ros-sim.sh" --prepare-only
python3 - <<'CHECK'
import json
from urllib.request import urlopen
with urlopen("http://127.0.0.1:18081/readyz", timeout=3) as response:
    assert json.load(response)["ready"], "Existing Qwen policy is not ready"
CHECK
# Only the policy HTTP endpoint crosses the mock container's network boundary.
docker exec -d relaxed_clarke bash -lc '
exec flock -n /tmp/mock-qwen-export.lock \
    socat UNIX-LISTEN:/home/rosdev/ros2_ws/recording/mock-qwen-policy.sock,fork,unlink-early,mode=0600 \
    TCP4:127.0.0.1:18081 >> /tmp/mock-qwen-export.log 2>&1
'
docker exec -d "$container" bash -lc '
exec flock -n /tmp/mock-qwen-import.lock \
    socat TCP4-LISTEN:18001,bind=127.0.0.1,reuseaddr,fork \
    UNIX-CONNECT:/home/rosdev/ros2_ws/recording/mock-qwen-policy.sock \
    >> /tmp/mock-qwen-import.log 2>&1
'
# A fresh connection also waits for both relay listeners to finish starting.
docker exec -i "$container" python3 - <<'CHECK'
import json,time
from urllib.request import urlopen
for attempt in range(20):
    try:
        with urlopen("http://127.0.0.1:18001/readyz", timeout=2) as response:
            assert json.load(response)["ready"]
        break
    except (OSError, AssertionError):
        if attempt==19: raise
        time.sleep(.25)
print("Qwen CV/IK policy ready through the isolated HTTP relay.")
CHECK
echo 'Uses the existing BF16 Qwen model, three real cameras, and bolt_to_tray task context.'
echo 'Commands go only to the mock arm/gripper on domain 77; the physical scene will not follow mock motion.'
echo 'First Enter moves the mock arm to start; second Enter runs Qwen. Esc aborts; q quits while idle.'
[[ ${1:-} != --prepare-only ]] || exit 0
if docker exec "$container" pgrep -x policy-runner >/dev/null 2>&1; then
    echo 'Quit the existing mock runner with q while idle, then run this command again.' >&2
    exit 1
fi
exec docker exec -it "$container" \
    bash /home/rosdev/ros2_ws/tools/run-mock-driver-runner.sh \
    bolt_to_tray http://127.0.0.1:18001
