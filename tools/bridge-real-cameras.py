#!/usr/bin/env python3
"""Copy camera messages into the network-isolated mock container over a Unix socket."""
import argparse
import base64
import fcntl
import json
import os
import socket
import threading
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import CameraInfo, CompressedImage

ROLES = ("left", "right", "wrist")
KINDS = {"image": (CompressedImage, "image_raw/compressed"),
         "info": (CameraInfo, "camera_info")}


def topic(role, kind):
    return f"/{role}_camera/color/{KINDS[kind][1]}"


def spin(node):
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


def export(node, path, max_age):
    changed = threading.Condition()
    pending = {}

    def receive(message, role, kind):
        item = {"role": role, "kind": kind, "received_at": time.time(),
                "message": base64.b64encode(serialize_message(message)).decode()}
        with changed:
            pending[(role, kind)] = item
            changed.notify()

    subscriptions = [
        node.create_subscription(typ, topic(role, kind),
                                 lambda msg, r=role, k=kind: receive(msg, r, k),
                                 qos_profile_sensor_data)
        for role in ROLES for kind, (typ, _) in KINDS.items()
    ]
    threading.Thread(target=spin, args=(node,), daemon=True).start()
    path.unlink(missing_ok=True)
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server:
        server.bind(str(path))
        server.listen(1)
        server.settimeout(1)
        print("Exporting only camera images and intrinsics from domain 42", flush=True)
        while rclpy.ok():
            try:
                client, _ = server.accept()
            except socket.timeout:
                continue
            with client:
                client.settimeout(1)
                try:
                    while rclpy.ok():
                        with changed:
                            changed.wait_for(lambda: bool(pending), timeout=1)
                            batch = list(pending.values())
                            pending.clear()
                        for item in batch:
                            if time.time() - item["received_at"] <= max_age:
                                client.sendall(json.dumps(item, separators=(",", ":")).encode() + b"\n")
                except (OSError, BrokenPipeError):
                    pass
    del subscriptions


def import_frames(node, path, max_age):
    publishers = {(role, kind): node.create_publisher(typ, topic(role, kind),
                                                    qos_profile_sensor_data)
                  for role in ROLES for kind, (typ, _) in KINDS.items()}
    print("Importing only camera images and intrinsics into domain 77", flush=True)
    while rclpy.ok():
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
                client.settimeout(2)
                client.connect(str(path))
                with client.makefile("rb") as stream:
                    while rclpy.ok():
                        line = stream.readline(2 * 1024 * 1024)
                        if not line:
                            break
                        if not line.endswith(b"\n"):
                            raise ValueError("oversized camera packet")
                        item = json.loads(line)
                        key = (item["role"], item["kind"])
                        if key not in publishers:
                            raise ValueError("only camera topics are allowed")
                        if not 0 <= time.time() - item["received_at"] <= max_age:
                            continue
                        message = deserialize_message(base64.b64decode(item["message"], validate=True),
                                                      KINDS[key[1]][0])
                        publishers[key].publish(message)
        except (OSError, ValueError) as error:
            node.get_logger().warning(str(error), throttle_duration_sec=5)
            time.sleep(0.5)


def check(node, max_age):
    latest = {}

    def receive(message, role):
        latest[role] = (time.monotonic(), message)

    subscriptions = [node.create_subscription(
        CompressedImage, topic(role, "image"), lambda msg, r=role: receive(msg, r),
        qos_profile_sensor_data) for role in ROLES]
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if len(latest) == 3 and all(time.monotonic() - at < max_age for at, _ in latest.values()):
            print(json.dumps({role: {"jpeg_bytes": len(message.data),
                                    "frame_id": message.header.frame_id}
                              for role, (_, message) in latest.items()}), flush=True)
            return
    publishers = {role: node.count_publishers(topic(role, "image")) for role in ROLES}
    raise RuntimeError(f"Missing fresh mock-domain camera feeds; received {list(latest)}; "
                       f"discovered publishers {publishers}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("export", "import", "check"))
    parser.add_argument("--socket", type=Path,
                        default=Path("/home/rosdev/ros2_ws/recording/mock-cameras.sock"))
    parser.add_argument("--max-age", type=float, default=0.25)
    args = parser.parse_args()
    expected = "42" if args.mode == "export" else "77"
    if os.environ.get("ROS_DOMAIN_ID") != expected:
        parser.error(f"{args.mode} requires ROS domain {expected}")
    # Both ends need UDP: camera restarts can strand existing LOCALHOST SHM readers.
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "SYSTEM_DEFAULT"
    os.environ["FASTDDS_DEFAULT_PROFILES_FILE"] = str(Path(__file__).with_name("mock-fastdds.xml"))
    guard = None
    if args.mode != "check":
        lock = args.socket.with_suffix(".lock") if args.mode == "export" else Path("/tmp/mock-cameras-import.lock")
        lock.parent.mkdir(parents=True, exist_ok=True)
        guard = lock.open("a")
        try:
            fcntl.flock(guard, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print(f"Camera {args.mode} is already running", flush=True)
            return
    rclpy.init()
    node = rclpy.create_node(f"mock_camera_{args.mode}")
    try:
        if args.mode == "export":
            export(node, args.socket, args.max_age)
        elif args.mode == "import":
            import_frames(node, args.socket, args.max_age)
        else:
            check(node, args.max_age)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if guard:
            guard.close()


if __name__ == "__main__":
    main()
