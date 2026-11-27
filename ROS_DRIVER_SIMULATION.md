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

Mock camera import/check processes and newly opened runner/RViz processes load
`tools/mock-fastdds.xml`, which allows only UDP on loopback. They set
`ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT` so Kilted's LOCALHOST override
cannot add shared-memory transport after the XML profile is loaded. Merely
setting `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` was ineffective with that override.
Docker networking remains disabled; the XML also restricts the transport to
127.0.0.1. Existing controller processes do not need to be restarted.

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

## Existing Qwen CV/IK policy with the mock ROS driver

Quit an existing mock runner with `q` while idle, then run on Spark:

```bash
bash ~/run-rc-qwen-ros-sim.sh
```

First Enter moves the mock arm to start; second Enter starts Qwen. Esc aborts
and holds; q quits while idle. RViz can stay open. This uses the existing BF16
Qwen CV/IK server and the three real cameras; it does not quantize or reload the
model. The three-view adapter adds `bolt_to_tray`'s instruction and captured
wrist goal image. The real scene will not respond to mock movement, so this
checks the integration and cannot demonstrate closed-loop task completion.

The launcher first prepares the existing isolated mock driver and camera relay.
A locked `socat` relay exposes only the existing policy HTTP server through a
filesystem Unix socket. Inside the mock container it is available at
`http://127.0.0.1:18001`; the host adapter remains `127.0.0.1:18081`, forwarding
to Qwen on `127.0.0.1:18080`. Docker networking stays disabled and no physical
devices are mounted in the mock container. Joint and gripper commands remain
in the mock ROS domain. The model service is shared with the real-arm adapter;
use one policy episode at a time to avoid competing for its planner state.

`bash ~/run-rc-qwen-ros-sim.sh --prepare-only` checks the setup without opening
a runner. Logs are `/tmp/mock-qwen-export.log` in `relaxed_clarke` and
`/tmp/mock-qwen-import.log` in `rc-ros-driver-sim`.

Validation on Spark: an inference request through the isolated relay, using mock
joint state and three real camera views, returned a valid bounded joint-delta
action in 4.553 seconds, with no planner fallback. Three subsequent control
requests took 18.4, 4.3 and 4.2 ms, below the runner's 100 ms control timeout.
These API checks published no motion commands and are not
a completed runner episode or a calibrated real-arm test.

## Cargo and the isolated mock container

The host scripts run the same `target/release/policy-runner` binary produced by
Cargo. Build in the writable devcontainer (`relaxed_clarke`) with the ROS,
Robotiq and Orbbec setup files sourced. The mock container mounts the checkout
read-only and has no network access or Cargo registry cache; it runs that built
binary. A bare `cargo run` there is not a supported build setup.

The scripts also prepare the driver/camera relay and select mock domain 77 and
the Qwen endpoint. `--task=bolt_to_tray` alone supplies only the task ID; the
adapter adds its configured instruction and goal image.

Transport regression validation: 12 independent fresh camera checks passed, then
one reader received 1849 left, 1850 right and 1795 wrist frames over 60 seconds.
Maximum observed inter-frame gap was 37 ms. The relay and the sustained reader
had no Fast DDS shared-memory file handles; the container retained network mode
`none`. The host launcher then opened the idle runner with all three camera
statuses live. This is a bounded check, not a long-term reliability guarantee.
