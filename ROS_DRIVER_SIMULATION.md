# ROS driver simulation on Spark

On Spark, open the mock driver runner with:

```bash
bash ~/run-rc-ros-sim.sh
```

The dedicated `rc-ros-driver-sim` container runs on ROS domain 77 with Docker
networking disabled and no physical devices mounted. The real stack remains on
domain 42. Both the UR12e and gripper use `mock_components/GenericSystem`; the
same ROS joint trajectory controller and Rust policy runner run against them.
The configured robot IP is loopback; duplicate USB camera drivers and hand-guiding
are not launched. The existing USB camera processes remain the sole device owners.

First Enter moves to the saved start pose. Second Enter starts the deterministic
wrist/gripper test policy. Esc aborts and holds; q quits while idle. The runner uses live right-camera images at their native 640x360 resolution.
The left, right and wrist image and CameraInfo feeds are copied one way from real
ROS domain 42 into mock domain 77 through a Unix socket. Image bytes, timestamps
and intrinsics are preserved; no joint states or commands cross that bridge.
This launcher continues to use the deterministic test policy and does not load Qwen.

Verified integration checks:

- The runner commanded the saved start pose; observed maximum joint error was 0.
- The test episode generated 145 trajectory messages and about 0.02 rad wrist motion.
- The mock finger moved from 0 to 0.695 rad.
- Esc ended the episode and held position, with 0 measured drift over 0.6 seconds.

These are ROS/controller integration checks. GenericSystem mirrors commands into
joint feedback; it does not model contact, collision, motor dynamics or the UR
network protocol. The separate `~/run-rc-policy-sim.sh` command tests the BF16
policy and three-view translation function with MuJoCo physics.

The interactive runner log is `/tmp/ros-driver-ui.log` inside the mock container;
the policy request log is `/tmp/mock-policy.log`, and `docker logs rc-ros-driver-sim`
shows driver startup and controller messages. The completed integration report
and terminal capture are under `recording/ros-driver-real-camera-check/`.

To stop the mock setup:

```bash
docker stop rc-ros-driver-sim
```

This affects only the mock container.

The camera bridge logs are `/tmp/mock-camera-export.log` in `relaxed_clarke` and
`/tmp/mock-camera-import.log` in the mock container. Exporter and importer locks
prevent duplicate relay processes. The launcher checks that all three image feeds
are fresh before opening the runner. Restart an older runner window to use the
updated camera arguments.

An additional episode test with real cameras verifies the runner accepts the
native 640x360 JPEGs. The generic image checker still defaults to 640x480; the
runner has an explicit `--camera-size WIDTHxHEIGHT` option, and the local Spark
build and three-camera launchers select 640x360. Frames are not resized.

Mock camera readers use Fast DDS UDP on loopback to avoid stale shared-memory
transport state. Docker networking remains disabled for this container.

## Robot and camera viewer

With the mock driver already running, open a second terminal on Spark and run:

```bash
bash ~/run-rc-ros-rviz.sh
```

RViz opens on the Spark desktop and can be watched through Moonlight. It shows
the mock UR12e and gripper plus the live left, right and wrist camera images.
The camera panels are independent images, not calibrated projections onto the
mock robot. RViz is only a viewer; use the policy runner terminal to start or
abort an episode. Closing RViz leaves the mock driver running.

The host launcher is preserved in `tools/spark/run-rc-ros-rviz.sh`; its display
configuration is `robot/mock-view.rviz`. Run RViz inside the existing isolated
mock container so it shares the driver's ROS discovery. A filesystem link to the
local X11 socket provides desktop output without enabling Docker networking;
the current Spark layout puts `/tmp` and the checkout on the same filesystem.
Desktop authentication is copied into the container's `/tmp` with user-only
permissions and is not stored in the repository. The launcher expects Spark's
active X11 desktop on display `:0`.

Verified on Spark: RViz's global status is OK, the arm and gripper render, and
all three image panels receive live compressed camera frames. The viewer has no
trajectory or gripper command publishers.
