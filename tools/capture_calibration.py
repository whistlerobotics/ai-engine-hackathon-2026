#!/usr/bin/env python3
"""Capture camera calibration and robot state without publishing commands."""
import argparse
import json
from pathlib import Path
import time


def main():
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, JointState
    from tf2_msgs.msg import TFMessage
    from geometry_msgs.msg import PoseStamped
    from rosidl_runtime_py.convert import message_to_ordereddict

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("rc_calibration_capture")
    captured = {"cameras": {}, "transforms": {}}
    received = {}
    subscriptions = []

    def store(key, message):
        captured[key] = message_to_ordereddict(message)
        received[key] = time.monotonic()

    for role in ("left", "right", "wrist"):
        def camera(message, role=role):
            captured["cameras"][role] = message_to_ordereddict(message)
            received[role] = time.monotonic()
        subscriptions.append(node.create_subscription(
            CameraInfo, f"/{role}_camera/color/camera_info", camera, qos_profile_sensor_data))
    for topic, key in (("/joint_states", "arm"), ("/gripper/joint_states", "gripper")):
        subscriptions.append(node.create_subscription(
            JointState, topic, lambda message, key=key: store(key, message), qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(
        PoseStamped, "/tcp_pose_broadcaster/pose", lambda message: store("tcp", message),
        qos_profile_sensor_data))

    def transforms(message):
        for transform in message.transforms:
            key = transform.header.frame_id + "->" + transform.child_frame_id
            captured["transforms"][key] = message_to_ordereddict(transform)
    subscriptions.append(node.create_subscription(TFMessage, "/tf", transforms, 100))
    static_qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions.append(node.create_subscription(TFMessage, "/tf_static", transforms, static_qos))
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    captured["captured_at_unix"] = time.time()
    captured["receipt_age_seconds"] = {
        key: time.monotonic() - stamp for key, stamp in received.items()}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(captured, indent=2))
    print(json.dumps({"output": str(args.output), "cameras": list(captured["cameras"]),
                      "states": list(received), "transforms": len(captured["transforms"])}))
    node.destroy_node()
    rclpy.shutdown()
    del subscriptions


if __name__ == "__main__":
    main()
