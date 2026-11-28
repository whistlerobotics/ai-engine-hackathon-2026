#!/usr/bin/env python3
"""Stage and review a hand-guided capture and replay."""

import argparse
from datetime import datetime, timedelta, timezone
from http.client import HTTPConnection, HTTPException
import json
import math
import os
from pathlib import Path
from queue import Empty, SimpleQueue
import re
import runpy
import shutil
import signal
import socket
import subprocess
import tempfile
from threading import Thread
from time import monotonic, sleep

from demo_runner import RealRunner
from demo_tui_view import run


ROOT = Path(__file__).resolve().parents[1]
MOCK_DOMAIN = "77"
CAPTURE_DOMAIN = "42"
MOCK_JOINT_LIMITS = (math.tau, math.tau, math.pi, math.tau, math.tau, math.tau)
# Match policy-runner's real start pose and worker tolerance.
PHYSICAL_HOME = (-3.1477, -1.6507, -2.4943, -0.6170, 1.4267, -4.5194)
HOME_TOLERANCE = 0.02
TASK_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]*")
EPISODE_ID = re.compile(r"ep_[0-9]{8}_[0-9]{6}")
CAMERA_TOPICS = {
    f"/{view}_camera/color/image_raw/compressed" for view in ("left", "right", "wrist")
}
HOLD_BUTTON = "Hold the hand-guiding button to record. Release it to save the take."
BAG_TOPICS = {
    "/joint_states",
    "/gripper/joint_states",
    "/joint_trajectory_controller/controller_state",
}


def timestamp():
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def write_json(path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")


def job_environment(domain):
    env = os.environ.copy()
    env.update(NO_COLOR="1", RCUTILS_COLORIZED_OUTPUT="0", PYTHONUNBUFFERED="1")
    if domain is not None:
        env["ROS_DOMAIN_ID"] = domain
    if domain == MOCK_DOMAIN:
        env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "SYSTEM_DEFAULT"
        env["FASTDDS_DEFAULT_PROFILES_FILE"] = str(ROOT / "tools" / "mock-fastdds.xml")
    return env


class Job:
    def __init__(self, command, log, *, domain=None, stdin=False):
        self.process = subprocess.Popen(
            command,
            cwd=ROOT,
            env=job_environment(domain),
            stdin=subprocess.PIPE if stdin else subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            errors="replace",
            start_new_session=True,
        )
        self.events = SimpleQueue()
        self.reader = Thread(target=self.read, args=(log,), daemon=True)
        self.reader.start()

    def read(self, log):
        with self.process.stdout as output, log.open("w") as stream:
            for line in output:
                stream.write(line)
                stream.flush()
                self.events.put(line.strip())

    def lines(self):
        while True:
            try:
                yield self.events.get_nowait()
            except Empty:
                return

    def send(self, command):
        self.process.stdin.write(command + "\n")
        self.process.stdin.flush()

    def interrupt(self, signum=signal.SIGINT):
        if self.process.poll() is None:
            try:
                os.killpg(self.process.pid, signum)
            except ProcessLookupError:
                pass

    def stop(self):
        self.interrupt()
        try:
            self.process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            self.interrupt(signal.SIGTERM)
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.interrupt(signal.SIGKILL)
                self.process.wait()
        if self.process.stdin is not None:
            self.process.stdin.close()
        self.reader.join(timeout=2)


class Demo:
    title = "Demo recorder"

    def __init__(self, args):
        self.args = args
        self.task = args.task
        self.task_choices = args.task_choices
        self.episode_id = ""
        self.observed_at = None
        self.finished_at = None
        self.root = args.out_dir.resolve()
        self.out_dir = str(self.root)
        self.stage = "capture"
        self.status = ""
        self.error = ""
        self.detail = []
        self.jobs = {}
        self.draft = None
        self.operation = None
        self.saved = None
        self.started = None
        self.deadline = None
        self.bag_topics = set()
        self.replay_recorder_ready = False
        self.bag_finished = False
        self.policy_port = None
        self.runner_phase = None
        self.runner_report = ""
        self.hand_guiding_released_at = None
        self.physical_started = False
        self.replay_stop_requested = False
        self.latest_replay_step = None
        # Imported data is already captured. In preview, simulate the operator gate.
        self.operator_ready = args.input is not None
        self.reset()

    @property
    def busy(self):
        return self.operation is not None

    @property
    def has_draft(self):
        return self.draft is not None

    @property
    def drives_arm(self):
        return getattr(self.args, "drive_replay", False)

    @property
    def physical_replay(self):
        return self.drives_arm or self.args.observe_replay

    @property
    def actions(self):
        if self.busy:
            actions = {}
            if self.operation == "preview-wait":
                actions["space"] = "Hold demo button"
            elif self.operation == "preview-guiding":
                actions["space"] = "Release demo button"
            elif self.operation == "observing":
                actions["b"] = "Mark external replay start"
            elif self.operation == "observed-running":
                actions["f"] = "Finish capture"
            elif self.drives_arm and "runner" in self.jobs:
                actions["abort"] = "Stop arm sequence"
            return actions | {"d": "Discard draft"}
        if self.stage == "capture":
            if self.args.input:
                return {"c": "Load hand-guided take"}
            if self.operator_ready:
                actions = {"c": "Arm recording"}
            elif self.drives_arm or self.args.preview:
                actions = {"h": "Move arm to home"}
            else:
                actions = {"h": "Verify start with Whistle runner" if self.args.observe_replay else "Confirm start pose"}
            if self.has_draft:
                actions["d"] = "Discard draft"
            else:
                actions["t"] = "Change task"
            return actions
        if self.stage == "home":
            action = (
                "Home arm and record replay" if self.drives_arm else
                "Arm no-human capture" if self.args.observe_replay else "Home mock and replay"
            )
            return {"r": action, "d": "Discard draft"}
        if self.stage == "review":
            return {"s": "Save pair", "r": "Redo replay", "d": "Discard draft"}
        if self.stage == "saved":
            return {"n": "New demo"}
        return {"d": "Discard draft"}

    def reset(self):
        self.stage = "capture"
        self.status = (
            "Load an existing hand-guided JSON."
            if self.args.input
            else "Home the arm in the cleared cell before arming the recorder."
            if self.drives_arm
            else "Verify the cell operator’s start pose before recording."
            if self.physical_replay
            else "Confirm the guided take is at its start pose."
        )
        self.detail = [
            "Preview • sample data only"
            if self.args.preview
            else "Human take + physical replay capture"
            if self.physical_replay
            else "Human take + isolated mock replay",
        ]
        self.episode_id = ""
        self.error = ""
        self.operator_ready = self.args.input is not None
        self.saved = None

    def dispatch(self, key, value=None):
        if key not in self.actions:
            return
        self.error = ""
        try:
            if key == "t":
                if value not in self.task_choices:
                    raise ValueError("Choose a task from the manifest.")
                if value != self.task:
                    self.task = value
                    self.reset()
            elif key == "h":
                if self.args.preview:
                    self.begin("preview-capture-home", 0.8)
                    self.status = "Moving the sample arm to home…"
                    return
                if self.drives_arm:
                    self.home_for_capture()
                    return
                if self.args.observe_replay:
                    self.run_operator(
                        ["mise", "run", "policy-runner", "--", "--mode", "real", "--operator-home-only"]
                    )
                self.operator_ready = True
                self.status = "Ready to arm the hand-guided recorder."
            elif key == "space":
                if self.operation == "preview-wait":
                    self.begin("preview-guiding")
                    self.status = "Recording the sample take. Release the hand-guiding button to save it."
                else:
                    self.finish_preview_capture()
            elif key == "c":
                self.capture()
            elif key == "r":
                self.replay()
            elif key == "b":
                self.mark_observed_start(timestamp())
            elif key == "f":
                self.finish_physical_recording()
            elif key == "abort":
                self.fail(RuntimeError("Arm sequence cancelled."))
            elif key == "s":
                self.save()
            elif key == "d":
                self.close()
                self.reset()
            elif key == "n":
                self.reset()
        except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
            self.fail(error)

    def begin(self, operation, seconds=None):
        self.operation = operation
        self.deadline = monotonic() + seconds if seconds is not None else None

    def create_draft(self):
        self.root.mkdir(parents=True, exist_ok=True)
        self.draft = Path(tempfile.mkdtemp(prefix=".draft-", dir=self.root))
        (self.draft / "logs").mkdir()

    def home_for_capture(self):
        if self.draft is None:
            self.create_draft()
        self.start_real_runner("http://127.0.0.1:1")
        self.begin("capture-home-start", 600)
        self.status = "Waiting for the physical homing runner…"

    def start_real_runner(self, policy_url):
        self.runner_phase = None
        self.runner_report = ""
        self.hand_guiding_released_at = None
        self.start_job(
            "hand-guiding",
            ["ros2", "topic", "echo", "/hand_guiding/active", "std_msgs/msg/Bool", "--field", "data"],
            domain=CAPTURE_DOMAIN,
        )
        self.jobs["runner"] = RealRunner(
            [
                "mise", "run", "policy-runner", "--", "--mode", "real",
                "--task", self.task, "--task-pack", str(self.args.task_pack), "--policy", policy_url,
            ],
            self.draft / "logs" / "runner.log", ROOT, job_environment(CAPTURE_DOMAIN),
        )

    def capture(self):
        if self.draft is None:
            self.create_draft()
        else:
            shutil.rmtree(self.draft / "capture", ignore_errors=True)
        self.started = timestamp()
        if self.args.input:
            shutil.copyfile(self.args.input, self.draft / "human.json")
            self.finish_capture()
        elif self.args.preview:
            self.begin("preview-wait")
            self.status = HOLD_BUTTON
        else:
            self.start_job("capture", self.record_command("capture"), domain=CAPTURE_DOMAIN)
            self.begin("capture")
            self.status = HOLD_BUTTON
            self.detail = ["Start with the hand-guiding button released."]

    def record_command(self, directory):
        return [
            "mise", "run", "record", "--",
            "--task", self.task, "--out-dir", str(self.draft / directory),
            "--camera-topic", self.args.camera_topic,
            "--camera-size", self.args.camera_size, "--once",
        ]

    def uses_operator_terminal(self, key):
        return self.args.observe_replay and key == "h"

    def start_job(self, name, command, **kwargs):
        self.jobs[name] = Job(command, self.draft / "logs" / f"{name}.log", **kwargs)

    def finish_capture(self):
        source = json.loads((self.draft / "human.json").read_text())
        if not isinstance(source, dict) or source.get("task_id") != self.task:
            raise ValueError(f"Recording task differs from {self.task!r}.")
        episode_id = source.get("episode_id")
        if not isinstance(episode_id, str) or EPISODE_ID.fullmatch(episode_id) is None:
            raise ValueError("The take needs its original ep_YYYYMMDD_HHMMSS episode ID.")
        steps = source.get("steps")
        if not isinstance(steps, list) or not 2 <= len(steps) <= 1201:
            raise ValueError("The hand-guided take needs 2–1,201 recorded samples.")
        for step in steps:
            observation = step.get("proprio") if isinstance(step, dict) else None
            if not isinstance(observation, dict):
                raise ValueError("The hand-guided take contains an invalid observation.")
            pose = observation.get("joint_pos")
            grip = step.get("gripper_command")
            if (
                not isinstance(pose, list)
                or len(pose) != 6
                or not all(
                    type(angle) in (int, float) and -limit <= angle <= limit
                    for angle, limit in zip(pose, MOCK_JOINT_LIMITS)
                )
                or type(grip) not in (int, float)
                or not 0.0 <= grip <= 1.0
            ):
                raise ValueError("The hand-guided take has joint or gripper data outside mock limits.")
        if source.get("ended_by") not in {"release", "time limit"}:
            raise ValueError(f"Hand-guided take ended with {source.get('ended_by')!r}. Capture it again.")
        self.capture_data = source
        self.episode_id = episode_id
        self.begin(None)
        self.stage = "home"
        self.status = (
            "Human take captured. Clear the cell before starting the recorded replay."
            if self.drives_arm else
            "Human take captured. Have the cell operator ready the existing replay."
            if self.args.observe_replay
            else "Human take captured. Mock replay will start at its first measured pose."
        )
        self.detail = [f"{len(steps)} human samples", f"Capture ended by {source['ended_by']}"]

    def replay(self):
        self.stage = "home"
        self.status = "Preparing replay…"
        if self.args.preview:
            self.begin("preview-home", 0.8)
            return
        if (self.draft / "no_human").exists():
            shutil.rmtree(self.draft / "no_human")
        replay_type = runpy.run_path(str(ROOT / "test-policy.py"))["Replay"]
        recording = self.draft / "human.json"
        playback = replay_type(recording)
        if self.drives_arm and max(
            abs(recorded - home) for recorded, home in zip(playback.start, PHYSICAL_HOME)
        ) > HOME_TOLERANCE:
            raise ValueError("The recorded start is outside the physical runner's home-pose tolerance.")
        self.replay_steps = len(playback.deltas)
        if not 1 <= self.replay_steps <= 1200:
            raise ValueError(f"Replay needs 1–1,200 steps; this take expands to {self.replay_steps}.")
        write_json(
            self.draft / "replay-plan.json",
            {
                "start_joint_pos": playback.start,
                "actions": [playback.delta(step) for step in range(self.replay_steps)],
            },
        )
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", 0))
            self.policy_port = sock.getsockname()[1]
        self.start_job(
            "policy",
            [
                "mise", "run", "test-policy", "--", "--bind", "127.0.0.1",
                "--port", str(self.policy_port), "--replay", str(recording), "--log-requests",
            ],
        )
        if self.physical_replay:
            self.observed_at = self.finished_at = None
            self.replay_recorder_ready = False
            (self.draft / "no_human.json").unlink(missing_ok=True)
            replay_capture = self.draft / "replay-capture"
            if replay_capture.exists():
                shutil.rmtree(replay_capture)
            self.start_job(
                "replay-record", [*self.record_command("replay-capture"), "--policy-take"],
                domain=CAPTURE_DOMAIN, stdin=True,
            )
            if self.args.observe_replay:
                self.start_bag()
                return
        if self.drives_arm:
            self.physical_started = False
            self.replay_stop_requested = False
            self.latest_replay_step = None
            self.start_real_runner(self.replay_policy_url())
            self.begin("runner-start", 600)
            self.status = "Waiting for the physical replay runner…"
            return
        self.start_job(
            "runner",
            [
                "mise", "run", "policy-runner", "--",
                "--mode", "mock", "--task", self.task, "--task-pack", str(self.args.task_pack),
                "--policy", self.replay_policy_url(), "--mock-replay-stdio",
                "--mock-replay-steps", str(self.replay_steps),
                "--mock-replay-start", *map(str, playback.start),
            ],
            domain=MOCK_DOMAIN, stdin=True,
        )
        self.begin("runner-start", 600)
        self.status = "Starting the isolated mock runner…"

    def run_operator(self, command):
        environment = {**os.environ, "ROS_DOMAIN_ID": CAPTURE_DOMAIN}
        with subprocess.Popen(command, cwd=ROOT, env=environment) as operator:
            while operator.poll() is None:
                self.refresh()
                sleep(0.02)
            if operator.returncode:
                raise subprocess.CalledProcessError(operator.returncode, command)

    def mark_observed_start(self, started_at):
        self.jobs["replay-record"].send("start")
        self.observed_at = started_at
        self.begin("observed-running")
        self.status = "Both recorders are capturing. Press F after the cell operator has finished."

    def finish_physical_recording(self):
        self.finished_at = timestamp()
        recorder = self.jobs.get("replay-record")
        if recorder is not None and recorder.process.poll() is None:
            try:
                recorder.send("finish")
            except BrokenPipeError:
                # The recorder can reach its duration limit as replay finishes.
                pass
        self.close_bag()

    def replay_policy_url(self):
        return f"http://127.0.0.1:{self.policy_port}"

    def replay_policy_ready(self):
        connection = HTTPConnection("127.0.0.1", self.policy_port, timeout=0.15)
        try:
            connection.request("GET", "/readyz")
            response = connection.getresponse()
            if response.status != 200:
                return False
            status = json.load(response)
            return (
                status.get("ready") is True
                and status.get("contract") == "v0"
                and status.get("schema_version") == 3
            )
        except (OSError, ValueError, HTTPException):
            return False
        finally:
            connection.close()

    def start_bag(self):
        self.stage = "replay"
        self.status = "Waiting for the no-human recorder to subscribe…"
        self.bag_topics.clear()
        self.bag_finished = False
        command = ["bash", "tools/record-three-cameras.sh"]
        if not self.physical_replay:
            command.append("--mock-state-only")
        self.start_job(
            "bag",
            [*command, str(self.draft / "no_human")],
            domain=CAPTURE_DOMAIN if self.physical_replay else MOCK_DOMAIN,
        )
        self.begin("bag-start", 180 if self.physical_replay else 35)

    def required_topics(self):
        return BAG_TOPICS | CAMERA_TOPICS if self.physical_replay else BAG_TOPICS

    def close_bag(self):
        self.jobs["bag"].interrupt()
        self.begin("bag-close", 20)
        self.status = "Closing the no-human take…"

    def handle_line(self, name, line):
        if not line:
            return
        if name == "capture":
            if line.startswith("Recording "):
                self.status = "Recording the hand-guided take. Release the button to save it."
            self.detail = [line]
        elif name == "replay-record":
            if line == "READY":
                self.replay_recorder_ready = True
            elif line == "STARTED":
                self.physical_started = True
        elif name == "hand-guiding" and line in {"false", "true"}:
            self.hand_guiding_released_at = monotonic() if line == "false" else None
            if line == "true" and self.operation not in {"capture-home-start", "runner-start"}:
                raise RuntimeError("Hand-guiding became active during the arm sequence.")
        elif name == "policy" and self.operation == "observing":
            request = re.fullmatch(r"([0-9]+\.[0-9]+) /act step 0", line)
            if request:
                age = max(0.0, monotonic() - float(request.group(1)))
                started = datetime.now(timezone.utc) - timedelta(seconds=age)
                self.mark_observed_start(started.isoformat(timespec="milliseconds"))
        elif name == "policy" and self.drives_arm and self.operation == "physical-replay":
            request = re.fullmatch(r"[0-9]+\.[0-9]+ /act step ([0-9]+)", line)
            if request:
                self.latest_replay_step = int(request[1])
        elif name == "runner":
            if self.drives_arm:
                if line in {"IDLE", "HOMING", "HOMED", "REPLAYING"}:
                    self.runner_phase = line
                if line.startswith(f"{self.task}: "):
                    self.runner_report = line
                return
            if line.startswith("ERROR"):
                raise RuntimeError(line.removeprefix("ERROR").strip() or "Mock runner failed.")
            if line == "READY" and self.operation == "runner-start":
                self.jobs["runner"].send("home")
                self.begin("homing", 90)
                self.status = "Moving the mock arm to the recorded start pose…"
            elif line == "HOMED" and self.operation == "homing":
                self.start_bag()
            elif line == "REPLAYED" and self.operation == "replaying":
                self.jobs["runner"].send("quit")
                self.close_bag()
        elif name == "bag" and self.operation == "bag-start":
            topic = re.search(r"Subscribed to topic ['\"]([^'\"]+)", line)
            if topic:
                self.bag_topics.add(topic.group(1))
            if self.required_topics() <= self.bag_topics and not self.physical_replay:
                self.jobs["runner"].send("run")
                self.begin("replaying", 100)
                self.status = "Replaying and capturing mock joint and controller states…"

    def finish_replay(self):
        self.begin(None)
        self.stage = "review"
        self.status = "Both takes are staged. Save or discard the pair."
        replay = (
            "sample data" if self.args.preview else
            "three-camera physical replay capture" if self.physical_replay else
            "mock ROS state and replay plan"
        )
        self.detail = [f"Human • {len(self.capture_data['steps'])} samples", f"No human • {replay}"]

    def refresh(self):
        if not self.busy:
            return
        try:
            self.poll()
        except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
            self.fail(error)

    def poll(self):
        if self.operation.startswith("preview-"):
            if self.deadline is not None and monotonic() >= self.deadline:
                self.advance_preview()
            return
        for name, job in list(self.jobs.items()):
            finished = job.process.poll()
            if finished is not None:
                job.reader.join(timeout=0.15)
            for line in job.lines():
                self.handle_line(name, line)
            if finished is None:
                continue
            if name == "capture" and finished == 0:
                self.collect_recording("capture", "human.json")
                self.finish_capture()
                self.stop_jobs()
                return
            if name == "replay-record" and finished == 0 and self.observed_at is not None:
                self.collect_recording("replay-capture", "no_human.json")
                if self.operation == "observed-running":
                    self.status = "Rust take staged. Press F when the cell operator finishes."
                job.stop()
                del self.jobs[name]
                continue
            if name == "runner" and self.drives_arm and finished == 0:
                if self.operation == "capture-home-close":
                    self.stop_jobs()
                    self.operator_ready = True
                    self.begin(None)
                    self.status = "Arm homed. Ready to arm the hand-guided recorder."
                    return
                if self.operation == "physical-replay-close":
                    self.verify_runner_report()
                    job.stop()
                    del self.jobs[name]
                    self.finish_physical_recording()
                    continue
            if name == "runner" and self.operation == "bag-close" and finished == 0:
                continue
            if name == "bag" and self.operation == "bag-close":
                self.verify_bag()
                self.bag_finished = True
                job.stop()
                del self.jobs[name]
                continue
            raise RuntimeError(f"{name.title()} exited with status {finished}. See logs/{name}.log.")
        if self.operation == "bag-close" and self.bag_finished:
            if not self.physical_replay or (self.draft / "no_human.json").is_file():
                self.stop_jobs()
                self.finish_replay()
                return
        if self.drives_arm and "runner" in self.jobs:
            self.poll_real_runner()
        if self.operation == "bag-start" and self.args.observe_replay:
            if (
                self.required_topics() <= self.bag_topics
                and self.replay_recorder_ready
                and self.replay_policy_ready()
            ):
                self.begin("observing")
                self.status = "Replay source and capture ready. The cell operator can now start replay."
                self.detail = [
                    f"Cell operator's policy source: {self.replay_policy_url()}",
                    f"Replay: {self.episode_id} · {self.replay_steps} action steps",
                ]
                return
        if self.deadline is not None and monotonic() >= self.deadline:
            pending = sorted(self.required_topics() - self.bag_topics) if self.operation == "bag-start" else []
            if self.operation == "bag-start" and self.physical_replay and not pending:
                pending = ["replay policy readiness" if self.replay_recorder_ready else "Rust recorder readiness"]
            suffix = f": {', '.join(pending)}" if pending else ""
            raise RuntimeError(f"Timed out during {self.operation}{suffix}.")

    def poll_real_runner(self):
        runner = self.jobs["runner"]
        hand_guiding_released = (
            self.hand_guiding_released_at is not None
            and monotonic() - self.hand_guiding_released_at < 0.75
        )
        if self.operation in {"capture-home-start", "runner-start"}:
            if self.runner_phase == "IDLE" and hand_guiding_released:
                runner.send("home")
                self.begin("capture-homing" if self.operation == "capture-home-start" else "homing", 90)
                self.status = "Moving the physical arm to home…"
            return
        if not hand_guiding_released:
            raise RuntimeError("Lost fresh confirmation that hand-guiding is released.")
        if self.operation == "capture-homing" and self.runner_phase == "HOMED":
            runner.send("quit")
            self.begin("capture-home-close", 10)
        elif self.operation == "homing" and self.runner_phase == "HOMED":
            self.start_bag()
        elif self.operation == "bag-start" and (
            self.required_topics() <= self.bag_topics
            and self.replay_recorder_ready and self.replay_policy_ready()
        ):
            self.jobs["replay-record"].send("start")
            self.observed_at = timestamp()
            self.begin("physical-record-start", 10)
            self.status = "Waiting for the Rust recorder to start sampling…"
        elif self.operation == "physical-record-start" and self.physical_started:
            if self.runner_phase != "HOMED":
                raise RuntimeError("The policy runner left its homed state before replay.")
            runner.send("run")
            self.begin("physical-replay-start", 10)
            self.status = "Starting physical replay…"
        elif self.operation == "physical-replay-start" and self.runner_phase == "REPLAYING":
            self.begin("physical-replay", 155)
            self.status = "Replaying and capturing the physical take…"
        elif self.operation == "physical-replay":
            if self.runner_phase == "IDLE":
                runner.send("quit")
                self.begin("physical-replay-close", 10)
            elif (
                not self.replay_stop_requested and self.latest_replay_step is not None
                and self.latest_replay_step >= self.replay_steps + 1
            ):
                # The next request follows a full stationary step at the final recorded pose.
                runner.send("cancel")
                self.replay_stop_requested = True
                self.status = "Recorded actions complete. Waiting for the runner to hold…"

    def verify_runner_report(self):
        report = re.fullmatch(
            rf"{re.escape(self.task)}: (Step limit reached|Aborted); ([0-9]+) steps, .*",
            self.runner_report,
        )
        if report and (
            (report[1] == "Step limit reached" and int(report[2]) == 1200)
            or (report[1] == "Aborted" and self.replay_stop_requested and int(report[2]) > self.replay_steps)
        ):
            return
        raise RuntimeError(self.runner_report or "The physical runner did not report a completed replay.")

    def collect_recording(self, directory, name):
        source = self.draft / directory
        outputs = list((source / self.task).glob("*.json"))
        if len(outputs) != 1:
            raise RuntimeError("The recorder did not produce one staged JSON file.")
        outputs[0].rename(self.draft / name)
        shutil.rmtree(source)

    def verify_bag(self):
        bag = self.draft / "no_human"
        metadata = bag / "metadata.yaml"
        if not metadata.is_file() or not any(bag.glob("*.mcap")):
            raise RuntimeError("The replay did not close with MCAP data and ROS bag metadata.")
        if self.physical_replay:
            try:
                import yaml
            except ImportError as error:
                raise RuntimeError("PyYAML is required to verify the physical bag in the ROS devcontainer.") from error
            try:
                info = yaml.safe_load(metadata.read_text())["rosbag2_bagfile_information"]
                topics = {
                    entry["topic_metadata"]["name"]
                    for entry in info["topics_with_message_count"]
                    if entry["message_count"] > 0
                }
            except (KeyError, TypeError, yaml.YAMLError) as error:
                raise RuntimeError("The physical bag metadata could not be read.") from error
            if self.required_topics() - topics:
                raise RuntimeError("No samples captured for " + ", ".join(sorted(self.required_topics() - topics)))

    def finish_preview_capture(self):
        write_json(
            self.draft / "human.json",
            {
                "task_id": self.task,
                "episode_id": datetime.now().strftime("ep_%Y%m%d_%H%M%S"),
                "ended_by": "release",
                "steps": [
                    {"proprio": {"joint_pos": [0.01 * i] * 6}, "gripper_command": 1.0}
                    for i in range(24)
                ],
            },
        )
        self.finish_capture()

    def advance_preview(self):
        if self.operation == "preview-capture-home":
            self.operator_ready = True
            self.begin(None)
            self.status = "Sample arm homed. Ready to arm the hand-guided recorder."
        elif self.operation == "preview-home":
            self.stage = "replay"
            self.status = "Replaying and capturing sample mock data…"
            self.begin("preview-replay", 1.4)
        else:
            self.replay_steps = len(self.capture_data["steps"]) - 1
            write_json(self.draft / "no_human.json", {"steps": self.replay_steps, "preview": True})
            self.finish_replay()

    def save(self):
        folder = self.root / self.task
        folder.mkdir(exist_ok=True)
        destination = folder / self.episode_id
        legacy = folder / f"{self.episode_id}_human.json"
        if destination.exists() or legacy.is_symlink() or legacy.exists():
            raise ValueError(f"Episode {self.episode_id} already has a saved pair.")
        if self.args.preview:
            replay = {"path": "no_human.json", "source": "sample", "policy_steps": self.replay_steps}
        elif self.physical_replay:
            replay = {
                "path": "no_human.json",
                "source": "tui_physical_replay" if self.drives_arm else "operator_observed_physical_replay",
                "domain": int(CAPTURE_DOMAIN),
                "marked_start": self.observed_at,
                "marked_complete": self.finished_at,
                "camera_capture": True,
                "policy_steps": self.replay_steps,
                "planned_actions": "replay-plan.json",
                "camera_bag": "no_human",
            }
        else:
            replay = {
                "path": "no_human",
                "source": "isolated_mock_ros",
                "domain": int(MOCK_DOMAIN),
                "policy_steps": self.replay_steps,
                "camera_capture": False,
                "planned_actions": "replay-plan.json",
            }
        metadata = {
            "schema_version": 1,
            "task_id": self.task,
            "episode_id": self.episode_id,
            "preview": self.args.preview,
            "staged_at": self.started,
            "saved_at": timestamp(),
            "takes": {
                "human": {
                    "path": "human.json",
                    "source": "sample" if self.args.preview else "import" if self.args.input else "ros",
                    "samples": len(self.capture_data["steps"]),
                    "started_at": self.capture_data.get("started_at"),
                    "ended_by": self.capture_data["ended_by"],
                },
                "no_human": replay,
            },
        }
        write_json(self.draft / "demo.json", metadata)
        self.draft.rename(destination)
        try:
            legacy.symlink_to(Path(self.episode_id) / "human.json")
        except OSError:
            destination.rename(self.draft)
            raise
        self.saved = destination
        self.draft = None
        self.stage = "saved"
        self.status = "Saved the Human and No human takes."
        self.detail = [str(self.saved), f"Legacy replay path: {legacy.name}"]

    def fail(self, error):
        failed_capture = self.stage == "capture"
        self.stop_jobs()
        self.begin(None)
        self.error = str(error)
        if failed_capture:
            if self.drives_arm and not self.args.input:
                self.operator_ready = False
            self.status = "Capture failed. Start a new take when ready."
        elif self.stage in {"home", "replay"}:
            self.stage = "home"
            self.status = "The hand-guided take is still staged. Retry the no-human take or discard."

    def stop_jobs(self):
        for name in ("runner", "bag", "capture", "replay-record", "policy", "hand-guiding"):
            job = self.jobs.pop(name, None)
            if job is not None:
                job.stop()

    def close(self):
        self.stop_jobs()
        self.begin(None)
        if self.draft is not None:
            shutil.rmtree(self.draft)
            self.draft = None


def manifest_tasks(path):
    manifest = json.loads(path.read_text())
    if not isinstance(manifest, dict) or manifest.get("schema_version") != 3:
        raise ValueError(f"Task manifest {path} must use schema version 3.")
    revision = manifest.get("revision")
    if not isinstance(revision, str) or not revision.strip():
        raise ValueError(f"Task manifest {path} needs a revision.")
    entries = manifest.get("tasks")
    if not isinstance(entries, list) or not entries:
        raise ValueError(f"Task manifest {path} has no tasks.")
    task_ids = []
    for entry in entries:
        task_id = entry.get("task_id") if isinstance(entry, dict) else None
        if not isinstance(task_id, str) or TASK_ID.fullmatch(task_id) is None:
            raise ValueError(f"Task manifest {path} contains an invalid task ID.")
        if task_id in task_ids:
            raise ValueError(f"Task manifest {path} contains duplicate task {task_id!r}.")
        task_ids.append(task_id)
    return tuple(task_ids)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", help="Initial task ID from the manifest.")
    parser.add_argument("--preview", action="store_true", help="Use sample data without ROS or Cargo.")
    parser.add_argument("--input", type=Path, help="Use an existing human-guided episode JSON.")
    parser.add_argument("--out-dir", type=Path)
    replay_mode = parser.add_mutually_exclusive_group()
    replay_mode.add_argument(
        "--drive-replay", action="store_true",
        help="Use the existing real policy runner to home and drive the recorded replay.",
    )
    replay_mode.add_argument(
        "--observe-replay", action="store_true",
        help="Passively record the cell operator's physical replay with the existing three-camera script.",
    )
    parser.add_argument("--camera-topic", default="/right_camera/color/image_raw/compressed")
    parser.add_argument("--camera-size", help="Frame size, such as 640x360.")
    parser.add_argument(
        "--task-pack", type=Path, metavar="MANIFEST",
        default=os.environ.get("WHISTLE_TASK_PACK", ROOT / "task-packs/simulator/manifest.json"),
    )
    args = parser.parse_args()
    if args.preview and (args.input or args.observe_replay or args.drive_replay):
        parser.error("--preview uses sample data and cannot be combined with --input or physical replay.")
    args.task_pack = args.task_pack.resolve()
    try:
        args.task_choices = manifest_tasks(args.task_pack)
        if args.input:
            args.input = args.input.resolve()
            recording_task = json.loads(args.input.read_text())["task_id"]
            if args.task and args.task != recording_task:
                parser.error("--task must match the task ID in the imported recording.")
            args.task = recording_task
    except (OSError, KeyError, TypeError, ValueError) as error:
        parser.error(str(error))
    if args.task is None:
        args.task = "bolt_to_tray" if "bolt_to_tray" in args.task_choices else args.task_choices[0]
    if args.task not in args.task_choices:
        parser.error(f"Task {args.task!r} is not in the selected manifest.")
    if args.camera_size is None:
        args.camera_size = "640x480" if args.camera_topic.startswith("/camera/") else "640x360"
    if args.out_dir is None:
        args.out_dir = ROOT / "recording" / "demo-preview" if args.preview else ROOT / "recording"

    controller = Demo(args)
    try:
        run(controller)
    finally:
        controller.close()


if __name__ == "__main__":
    main()
