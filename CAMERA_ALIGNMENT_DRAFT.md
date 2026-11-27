# Draft: three-view camera adapter for Reality Check policies

At Whistle commit `66dd885`, the runner sends `images.top` as one 640×480 JPEG.
The Reality Check policy API expects three 640×360 JPEGs named `wrist`, `left`
and `right`. The cell has a Gemini 336 on each side and a wrist camera reported
as a Gemini 335Lg. Verify the wrist model and host connection, then bind each
device serial number to its camera role in the ROS launch configuration.

`tools/three_view_proxy.py` is an experimental adapter between the runner and
the policy. It subscribes to three explicitly configured ROS 2
`sensor_msgs/msg/CompressedImage` topics, checks that each latest frame has a
640×360 JPEG header, and replaces the runner's `images` object with the three
named frames. It forwards the rest of each observation and the policy's response
unchanged unless task context is configured below. Missing, malformed or stale frames prevent a policy request. It
never fabricates a camera view.

The adapter is deliberately outside the runner, so its existing contract and
single-camera operation are unchanged. It requires a ROS 2 Python environment
with `rclpy` and `sensor_msgs`, three real streams, and a reachable policy.
Topic names below are placeholders: use the topics for the actual installation.
They must each be distinct. Each should publish its own 640×360 JPEG, and the
physical camera position, orientation and intrinsics need to be checked against
the intended simulation view. The adapter neither resizes nor synchronizes
frames; freshness is measured from receipt, not camera capture time.

Example topology (use only after configuring the real topics and validating the
full workflow):

```sh
python3 tools/three_view_proxy.py --upstream http://127.0.0.1:8000 \
  --wrist-topic /wrist/color/image_raw/compressed \
  --left-topic /left/color/image_raw/compressed \
  --right-topic /right/color/image_raw/compressed
```

Set the runner policy URL to `http://127.0.0.1:8001`. The current runner still
requires its original 640×480 camera input; alternatively its `--no-camera`
mode can supply the runner's unused placeholder while the adapter independently
requires all three live streams. In that mode the runner's own camera status
will not show the adapter's cameras. The adapter must be supervised separately.

## Reality Check mock bridge

[Reality Check PR #144](https://github.com/w3joe/reality-check/pull/144) adds a
mock-only launcher that already sends a schema-v2 observation with three sample
images. Point its `--policy-url` at the proxy URL reachable from the mock
container. Keep the existing observation template. The proxy replaces its
three images and passes its instruction, goal, and mock proprioception through
with the policy response and step echo headers. For synthetic camera tests,
publish three distinct 640×360 JPEG streams on the configured topics.

## Task context in the translation layer

The arm runner supplies robot state, task ID, episode ID and step. Configure the
proxy with an instruction file and a goal JPEG to add `instruction` and `goal`
without changing the arm interface. All three context options are required
together. A request for any other task ID is rejected before reaching the policy.
The files are loaded at proxy startup; restart the proxy after changing them.

On Joe Spark 2, the configured task is `bolt_to_tray`, the instruction is
“Move the silver bolt to the tray.” and the goal is the real wrist-camera photo
captured on 27 November. Its native 640×360 dimensions are preserved; the
640×480 simulation sample remains a separate file.

```sh
python3 tools/three_view_proxy.py --upstream http://127.0.0.1:18080 \
  --port 18081 --task bolt_to_tray \
  --instruction-file tasks/bolt_to_tray.txt \
  --goal-image tasks/bolt_to_tray.wrist-goal.jpg \
  --upstream-timeout 30 \
  --wrist-topic /wrist_camera/color/image_raw/compressed \
  --left-topic /left_camera/color/image_raw/compressed \
  --right-topic /right_camera/color/image_raw/compressed
```

The standalone `rc-three-view-proxy` Docker container runs this configuration;
`docker start rc-three-view-proxy` starts it again if stopped. It listens only
on localhost. It subscribes to camera images and calls the policy HTTP service;
it does not publish robot commands.

The proxy timeout defaults to one second for existing users. Qwen's initial
planning call needs a longer budget, hence 30 seconds above. The arm runner's
separate 100 ms timeout is unchanged and must be handled before using it with
this planner. This does not resolve camera extrinsics, palm-pose, starting-pose
or action-limit mismatches. The supplied policy's perception assumes simulation
camera geometry, including a fixed goal-camera pose; using a real wrist goal
still requires validation of that geometry before physical execution.

Run the standalone logic tests with:

```sh
python3 -m unittest discover -s tools -p 'test_three_view_proxy.py' -v
```
