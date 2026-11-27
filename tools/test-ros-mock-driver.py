#!/usr/bin/env python3
"""Exercise the Rust runner against isolated ros2_control mock hardware."""
import argparse
import ast
import errno
import json
import os
import pty
import re
import signal
import struct
import termios
import time
import fcntl
from pathlib import Path
from urllib.request import urlopen
from xml.etree import ElementTree

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=Path("/tmp/ros-driver-check"))
    args = parser.parse_args()
    assert os.environ.get("ROS_DOMAIN_ID") == "77", "test requires isolated ROS domain 77"
    os.environ["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"
    repo = Path("/home/rosdev/ros2_ws")
    source = (repo / "crates/policy-runner/src/worker.rs").read_text()
    start = ast.literal_eval(re.search(r"START_POSE: Joints = (\[[^;]+\]);", source)[1])
    with urlopen("http://127.0.0.1:18000/readyz", timeout=3) as response:
        assert json.load(response)["ready"]
    args.out.mkdir(parents=True, exist_ok=False)
    pid, terminal = pty.fork()
    if pid == 0:
        os.environ["TERM"] = "xterm-256color"
        os.execv(str(repo / "target/release/policy-runner"), [
            "policy-runner", "--task", "mock-driver-check",
            "--policy", "http://127.0.0.1:18000",
            "--camera-topic", "/right_camera/color/image_raw/compressed",
            "--camera-size", "640x360",
        ])
    fcntl.ioctl(terminal, termios.TIOCSWINSZ, struct.pack("HHHH", 35, 120, 0, 0))
    os.set_blocking(terminal, False)
    rclpy.init()
    node = rclpy.create_node("mock_driver_integration_check")
    names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    state = {"q": None, "finger": None, "plugins": {}, "commands": 0}
    capture = bytearray()
    episode_positions = []
    episode_fingers = []
    collecting = False

    def arm(message):
        state["q"] = [message.position[message.name.index(name)] for name in names]
        if collecting:
            episode_positions.append(state["q"])

    def finger(message):
        state["finger"] = message.position[message.name.index("finger_joint")]
        if collecting:
            episode_fingers.append(state["finger"])

    def description(message, key):
        root = ElementTree.fromstring(message.data)
        state["plugins"][key] = root.find(".//ros2_control/hardware/plugin").text.strip()

    def trajectory(_message):
        state["commands"] += 1

    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [
        node.create_subscription(JointState, "/joint_states", arm, 10),
        node.create_subscription(JointState, "/gripper/joint_states", finger, 10),
        node.create_subscription(JointTrajectory,
                                 "/joint_trajectory_controller/joint_trajectory", trajectory, 10),
        node.create_subscription(String, "/robot_description", lambda m: description(m, "arm"), latched),
        node.create_subscription(String, "/gripper/robot_description", lambda m: description(m, "gripper"), latched),
    ]

    def tick():
        rclpy.spin_once(node, timeout_sec=0.005)
        try:
            capture.extend(os.read(terminal, 65536))
        except BlockingIOError:
            pass
        except OSError as error:
            if error.errno != errno.EIO:
                raise

    def wait_for(condition, timeout, label):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            tick()
            if condition():
                return
        raise AssertionError(f"timed out: {label}; state={state}")

    def pause(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            tick()

    def readable_text():
        return re.sub(rb"\x1b\[[0-?]*[ -/]*[@-~]", b" ", capture).decode(errors="replace")

    try:
        wait_for(lambda: state["q"] is not None and state["finger"] is not None
                 and len(state["plugins"]) == 2, 8, "mock joint states and descriptions")
        assert set(state["plugins"].values()) == {"mock_components/GenericSystem"}
        pause(1)
        print("Both hardware plugins are GenericSystem on domain 77; moving mock arm to start.", flush=True)
        os.write(terminal, b"\r")
        wait_for(lambda: re.search(r"At\s+the\s+start\s+pose", readable_text()),
                 35, "runner start move")
        start_error = max(abs(a - b) for a, b in zip(state["q"], start))
        assert start_error <= 0.02, f"runner reported arrival with error {start_error}"
        commands_before = state["commands"]
        collecting = True
        os.write(terminal, b"\r")
        pause(7.2)
        os.write(terminal, b"\x1b")
        pause(1)
        collecting = False
        hold_position = list(state["q"])
        pause(0.6)
        hold_error = max(abs(a - b) for a, b in zip(state["q"], hold_position))
        excursion = max(abs(q[5] - start[5]) for q in episode_positions)
        assert state["commands"] - commands_before >= 130, state["commands"]
        assert excursion > 0.005, excursion
        assert max(episode_fingers) - min(episode_fingers) > 0.1, episode_fingers
        assert hold_error < 0.001, hold_error
        text = readable_text()
        assert "abort" in text.lower(), "runner did not report abort"
        os.write(terminal, b"q")
        pause(0.5)
        report = {
            "passed": True, "ros_domain_id": 77, "network": "none",
            "hardware_plugins": state["plugins"], "start_error_rad": start_error,
            "episode_trajectory_messages": state["commands"] - commands_before,
            "wrist_excursion_rad": excursion,
            "finger_range_rad": [min(episode_fingers), max(episode_fingers)],
            "hold_drift_rad_over_0_6s": hold_error,
            "policy": "deterministic test-policy.py, wrist amplitude 0.02 rad",
            "camera": "live right camera, 640x360, copied from domain 42",
            "scope": "Rust runner + ROS controllers + mock arm/gripper + abort/hold; no physical devices",
        }
        (args.out / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2), flush=True)
    finally:
        (args.out / "runner.typescript").write_bytes(capture)
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        os.waitpid(pid, 0)
        os.close(terminal)
        node.destroy_node()
        rclpy.shutdown()
        del subscriptions


if __name__ == "__main__":
    main()
