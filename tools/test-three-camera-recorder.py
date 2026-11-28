#!/usr/bin/env python3
"""Check the Rust recorder on domain 77 with synthetic robot state and supplied JPEGs.

Run in a network-isolated container without hardware devices. The source frames
may be captured real-camera JPEGs; this does not test physical hand-guiding.
"""

import argparse
import base64
import hashlib
import json
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Bool, Float64MultiArray

ROLES = ("wrist", "left", "right")
JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frames-dir", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()
    assert os.environ.get("ROS_DOMAIN_ID") == "77", "requires isolated domain 77"
    repo = Path(__file__).resolve().parent.parent
    frames = {role: (args.frames_dir / f"{role}.jpg").read_bytes() for role in ROLES}
    assert len({hashlib.sha256(frame).hexdigest() for frame in frames.values()}) == 3
    args.out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node("three_camera_recorder_test_source")
    cameras = {
        role: node.create_publisher(
            CompressedImage, f"/{role}_camera/color/image_raw/compressed", qos_profile_sensor_data
        )
        for role in ROLES
    }
    arm = node.create_publisher(JointState, "/joint_states", 10)
    finger = node.create_publisher(JointState, "/gripper/joint_states", 10)
    active = node.create_publisher(Bool, "/hand_guiding/active", 10)
    command = node.create_publisher(
        Float64MultiArray, "/gripper/finger_position_controller/commands", 10
    )

    def publish(guiding: bool, missing: str | None = None) -> None:
        stamp = node.get_clock().now().to_msg()
        for role, publisher in cameras.items():
            if role != missing:
                image = CompressedImage()
                image.header.stamp = stamp
                image.format = "jpeg"
                image.data = frames[role]
                publisher.publish(image)
        state = JointState()
        state.header.stamp = stamp
        state.name, state.position, state.velocity = JOINTS, [0.0] * 6, [0.0] * 6
        arm.publish(state)
        state = JointState()
        state.header.stamp = stamp
        state.name, state.position = ["finger_joint"], [0.2]
        finger.publish(state)
        active.publish(Bool(data=guiding))
        command.publish(Float64MultiArray(data=[0.2]))
        rclpy.spin_once(node, timeout_sec=0.005)

    def phase(seconds: float, guiding: bool, missing: str | None = None) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            publish(guiding, missing)
            time.sleep(0.02)

    reports = []
    try:
        for name, permissive, missing in [
            ("complete", False, None),
            ("strict_missing_left", False, "left"),
            ("permissive_missing_left", True, "left"),
        ]:
            log_path = args.out / f"{name}.log"
            with log_path.open("w") as log:
                argv = [
                    str(repo / "target/release/record"),
                    "--task",
                    name,
                    "--out-dir",
                    str(args.out),
                    "--max-duration",
                    "10",
                ]
                if permissive:
                    argv.append("--permissive")
                process = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT)
                try:
                    phase(1.0, False)
                    phase(1.2, True)
                    phase(0.5, True, missing)
                    phase(0.4, False)
                finally:
                    process.send_signal(signal.SIGINT)
                    process.wait(timeout=5)
            assert process.returncode == 0, log_path.read_text()
            files = list((args.out / name).glob("ep_*.json"))
            assert len(files) == 1, (files, log_path.read_text())
            episode = json.loads(files[0].read_text())
            assert episode["schema_version"] == 3
            assert len(episode["steps"]) >= 15, len(episode["steps"])
            replaced = 0
            for step in episode["steps"]:
                assert set(step["images"]) == set(ROLES)
                for role in ROLES:
                    image = base64.b64decode(step["images"][role], validate=True)
                    if image != frames[role]:
                        assert permissive and role == missing, (name, role)
                        assert image.startswith(b"\xff\xd8") and image.endswith(b"\xff\xd9")
                        replaced += 1
                assert len(step["proprio"]["joint_pos"]) == 6
                assert step["proprio"]["gripper"] is not None
            if permissive:
                assert replaced > 0 and episode["steps_without_image"] == replaced
            elif missing:
                assert f"/{missing}_camera/" in episode["ended_by"]
                assert episode["steps_without_image"] == 0
            else:
                assert episode["ended_by"] == "release"
                assert episode["steps_without_image"] == 0
            reports.append(
                {
                    "case": name,
                    "steps": len(episode["steps"]),
                    "ended_by": episode["ended_by"],
                    "replaced_views": replaced,
                    "file": str(files[0]),
                }
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()
    report = {
        "passed": True,
        "ros_domain_id": 77,
        "scope": "Rust recorder; supplied camera bytes and synthetic robot/button state",
        "source_sha256": {
            role: hashlib.sha256(frame).hexdigest() for role, frame in frames.items()
        },
        "cases": reports,
    }
    (args.out / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
