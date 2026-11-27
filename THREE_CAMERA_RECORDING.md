# Three-camera recording on joe-spark-2

The left, right and wrist cameras already run as independent ROS processes in the
devcontainer. Each physical camera must have only one driver process. Robot
bringup now defaults to `camera:=false` locally, so restarting it reuses those
feeds without opening the right camera a second time.

From Spark, start recording in a separate terminal:

```bash
docker exec -it relaxed_clarke bash -lc '
cd /home/rosdev/ros2_ws
bash tools/record-three-cameras.sh
'
```

Ctrl-C closes and saves the bag under `recording/three-camera-<UTC timestamp>`
in the checkout. This records continuously, including during a policy rollout;
it does not wait for the hand-guiding button and sends no robot commands.

The bag contains three JPEG image topics, camera intrinsics, joint and gripper
state, controller commands and feedback, speed scaling, robot status, TCP pose,
and TF. Image messages retain their camera timestamps; capture does not imply
hardware synchronization or calibrated camera-to-robot transforms.

The existing `target/release/record` tool still writes single-camera demonstration
JSON from `/camera/color/image_raw/compressed`. It does not record three cameras.
The new recorder writes ROS bags in MCAP format, rather than changing that JSON
schema. Use the new command to capture all three cameras during a run.

The current robot launch command, inside the sourced container from `robot/`, is:

```bash
ros2 launch launch.xml robot:=real camera:=false
```

If all separate camera processes have been stopped, the robot launch can instead
own all three cameras with `camera:=true three_view:=true`. Do not run both sets
of camera drivers at the same time. A single-camera setup can explicitly pass
`camera:=true three_view:=false`; its stream uses `/camera/...`.

## Policy runner with a live camera indicator

On Spark, quit the previous runner while idle, then use:

```bash
bash ~/run-rc-policy.sh bolt_to_tray
```

This starts the patched runner with
`--camera-topic /right_camera/color/image_raw/compressed --camera-size 640x360`
and the three-camera
translation proxy at `http://127.0.0.1:18081`. The runner displays and checks the
live right-camera feed; the proxy supplies left, right and wrist images to the
policy. No `--no-camera` flag is needed. The runner opens idle; this command does
not start a motion or episode.

The equivalent command inside the container is:

```bash
bash tools/run-three-camera-policy.sh bolt_to_tray
```

The local Spark build also defaults to the right-camera topic and policy proxy
port 18081. Restart an older runner process to pick up the new defaults. The
launcher above passes both settings explicitly.

## Policy and translation test in MuJoCo

On Spark:

```bash
bash ~/run-rc-policy-sim.sh
```

This reuses the dedicated `rc-policy-sim-engine` (port 18090) and BF16
`rc-policy-sim-model` (port 18082) containers. It resets `bolt_to_tray` at seed 0,
feeds three native 640x360 simulated views and the simulation task context through
`three_view_proxy.translate`, and applies returned joint deltas only to MuJoCo.
It sends no ROS messages or physical-robot commands. It tests the policy,
translation function and physics loop; it does not exercise the Rust ROS
trajectory controller or verify real-arm calibration.

Results go under `recording/sim-bolt-to-tray-<timestamp>-<pid>/`, with a JSON
report, step journal, first translated policy request, simulated goal and final
camera JPEGs. The first verified run completed successfully in 98 simulation
steps at seed 0. One successful seed is not a general task-success estimate.
