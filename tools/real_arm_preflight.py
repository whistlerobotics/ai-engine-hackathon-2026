#!/usr/bin/env python3
"""Inspect the running Spark real ROS graph without sending motion commands."""

import os
import re
import subprocess
from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor

TIMEOUT = 10
ROLES = ("wrist", "left", "right")


def sample(topic: str, message_type: str, field: str, *, latched: bool = False) -> tuple[str, ...]:
    qos = ("--qos-reliability", "reliable", "--qos-durability", "transient_local") if latched else (
        "--qos-reliability", "best_effort",
    )
    return (
        "topic", "echo", topic, message_type, "--once",
        *qos, "--field", field,
    )


def positive_speed(output: str) -> bool:
    values = [line.strip() for line in output.splitlines() if line.strip() not in ("", "---")]
    try:
        return len(values) == 1 and 0.0 < float(values[0]) <= 1.0
    except ValueError:
        return False


def probe(check: tuple[str, tuple[str, ...], str | Callable[[str], bool]]) -> bool:
    label, arguments, expected = check
    try:
        result = subprocess.run(
            ["ros2", *arguments], capture_output=True, text=True, timeout=TIMEOUT,
            check=False, env={**os.environ, "NO_COLOR": "1", "RCUTILS_COLORIZED_OUTPUT": "0"},
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        print(f"FAIL {label} ({error})")
        return False
    output = re.sub(r"\x1b\[[0-9;]*m", "", result.stdout)
    if callable(expected):
        matches = expected(output)
    elif arguments[0] == "control":
        matches = any(
            fields[0] == expected and fields[-1] == "active"
            for line in output.splitlines()
            if (fields := line.split())
        )
    else:
        matches = expected in {line.strip().casefold() for line in output.splitlines()}
    passed = result.returncode == 0 and matches
    message = "OK" if passed else "FAIL"
    print(f"{message} {label}")
    if not passed:
        details = result.stderr.strip() or result.stdout.strip()
        if details:
            print(details)
    return passed


def main() -> int:
    if os.environ.get("ROS_DOMAIN_ID") != "42":
        print("Set ROS_DOMAIN_ID=42 in the environment that can see Spark's real ROS graph.")
        return 2

    checks = [
        ("arm feedback", sample("/joint_states", "sensor_msgs/msg/JointState", "header"), "stamp:"),
        ("gripper feedback", sample(
            "/gripper/joint_states", "sensor_msgs/msg/JointState", "header"), "stamp:"),
        ("External Control program running", sample(
            "/io_and_status_controller/robot_program_running", "std_msgs/msg/Bool", "data", latched=True), "true"),
        ("active arm trajectory controller",
         ("control", "list_controllers", "-c", "/controller_manager"),
         "joint_trajectory_controller"),
        ("positive controller speed scaling", sample(
            "/speed_scaling_state_broadcaster/speed_scaling", "std_msgs/msg/Float64", "data"),
         positive_speed),
        ("active gripper controller",
         ("control", "list_controllers", "-c", "/gripper/controller_manager"),
         "finger_position_controller"),
    ]
    checks.extend(
        (f"{role} camera frames", sample(
            f"/{role}_camera/color/image_raw/compressed", "sensor_msgs/msg/CompressedImage", "header",
        ), "stamp:")
        for role in ROLES
    )
    with ThreadPoolExecutor() as executor:
        results = list(executor.map(probe, checks))
    print("ROS diagnostics only. Physical policy commissioning remains pending.")
    return 0 if all(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
