import importlib.util
from pathlib import Path
import subprocess
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location(
    "real_arm_preflight", Path(__file__).with_name("real_arm_preflight.py")
)
preflight = importlib.util.module_from_spec(spec)
spec.loader.exec_module(preflight)


class PreflightTests(unittest.TestCase):
    def test_speed_must_be_finite_positive_and_at_most_one(self):
        for value, expected in [
            ("0.0", False), ("-0.1", False), ("nan", False),
            ("inf", False), ("1.01", False), ("", False),
            ("not available", False), ("0.01\n", True), ("1.0", True),
            ("0.25\n---\n", True), ("0.0\n---\n", False),
            ("0.25\n---\n0.0\n---\n", False),
        ]:
            with self.subTest(value=value):
                self.assertEqual(preflight.positive_speed(value), expected)

    def test_probe_checks_speed_and_command_status(self):
        checks = [
            (("speed", ("topic", "echo"), preflight.positive_speed), "0.0\n", False),
            (("speed", ("topic", "echo"), preflight.positive_speed), "0.25\n", True),
            (("controller", ("control", "list_controllers"), "joint_trajectory_controller"),
             "joint_trajectory_controller type inactive\n", False),
            (("controller", ("control", "list_controllers"), "joint_trajectory_controller"),
             "joint_trajectory_controller type active\n", True),
            (("program", ("topic", "echo"), "true"), "false\n", False),
        ]
        for check, output, expected in checks:
            with self.subTest(output=output):
                result = subprocess.CompletedProcess([], 0, stdout=output, stderr="")
                with patch.object(preflight.subprocess, "run", return_value=result):
                    self.assertEqual(preflight.probe(check), expected)


if __name__ == "__main__":
    unittest.main()
