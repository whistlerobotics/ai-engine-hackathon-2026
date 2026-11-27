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
unchanged. Missing, malformed or stale frames prevent a policy request. It
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

The camera adapter **does not provide full API compatibility**. The runner at
the pinned revision does not send Reality Check's `instruction` and `goal`
fields. It also has separate palm-pose, action-limit and starting-pose
mismatches. Those require their own integration work; the adapter does not
invent missing context or transform pose or actions. In particular, a policy
requiring `instruction` or `goal` cannot run correctly using this change alone.

Before adopting this approach, agree the three physical feeds and placements,
source the task context and goal image, test the entire runner-to-policy path,
verify latency against the runner's 100 ms timeout, and decide how camera status
should appear to operators. No actual ROS camera or robot has been tested here.

Run the standalone logic tests with:

```sh
python3 -m unittest discover -s tools -p 'test_three_view_proxy.py' -v
```
