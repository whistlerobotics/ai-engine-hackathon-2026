#!/usr/bin/env python3
"""Stage and review a hand-guided capture and its isolated mock replay."""

import argparse
from datetime import datetime, timezone
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

from demo_tui_view import run


ROOT = Path(__file__).resolve().parents[1]
MOCK_DOMAIN = "77"
CAPTURE_DOMAIN = "42"
MOCK_JOINT_LIMITS = (math.tau, math.tau, math.pi, math.tau, math.tau, math.tau)
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


class Job:
    def __init__(self, command, log, *, domain=None, stdin=False):
        env = os.environ.copy()
        env.update(NO_COLOR="1", RCUTILS_COLORIZED_OUTPUT="0", PYTHONUNBUFFERED="1")
        if domain is not None:
            env["ROS_DOMAIN_ID"] = domain
        if domain == MOCK_DOMAIN:
            env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "SYSTEM_DEFAULT"
            env["FASTDDS_DEFAULT_PROFILES_FILE"] = str(ROOT / "tools" / "mock-fastdds.xml")
        self.process = subprocess.Popen(
            command,
            cwd=ROOT,
            env=env,
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
    def actions(self):
        if self.busy:
            actions = {}
            if self.operation == "preview-wait":
                actions["space"] = "Hold demo button"
            elif self.operation == "preview-guiding":
                actions["space"] = "Release demo button"
            elif self.operation == "observing":
                actions["b"] = "Mark replay start"
            elif self.operation == "observed-running":
                actions["f"] = "Mark replay complete"
            return actions | {"d": "Discard draft"}
        if self.stage == "capture":
            if self.args.input:
                return {"c": "Load hand-guided take"}
            actions = (
                {"c": "Arm recording"} if self.operator_ready else {"h": "Confirm start pose"}
            )
            if self.has_draft:
                actions["d"] = "Discard draft"
            else:
                actions["t"] = "Change task"
            return actions
        if self.stage == "home":
            action = "Arm no-human capture" if self.args.observe_replay else "Home mock and replay"
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
            else "Confirm the guided take is at its start pose."
        )
        self.detail = [
            "Preview • sample data only"
            if self.args.preview
            else "Human take + physical replay capture"
            if self.args.observe_replay
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
                self.observed_at = timestamp()
                self.begin("observed-running")
                self.status = "Recording the no-human take. Press F once the operator finishes the replay."
            elif key == "f":
                self.finished_at = timestamp()
                self.close_bag()
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

    def capture(self):
        if self.draft is not None:
            self.close()
        self.root.mkdir(parents=True, exist_ok=True)
        self.draft = Path(tempfile.mkdtemp(prefix=".draft-", dir=self.root))
        (self.draft / "logs").mkdir()
        self.started = timestamp()
        if self.args.input:
            shutil.copyfile(self.args.input, self.draft / "human.json")
            self.finish_capture()
        elif self.args.preview:
            self.begin("preview-wait")
            self.status = HOLD_BUTTON
        else:
            command = [
                "mise", "run", "record", "--",
                "--task", self.task, "--out-dir", str(self.draft / "capture"),
                "--camera-topic", self.args.camera_topic,
                "--camera-size", self.args.camera_size, "--once",
            ]
            self.start_job("capture", command, domain=CAPTURE_DOMAIN)
            self.begin("capture")
            self.status = HOLD_BUTTON
            self.detail = ["Start with the hand-guiding button released."]

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
            "Human take captured. Have the cell operator ready the existing replay."
            if self.args.observe_replay
            else "Human take captured. Mock replay will start at its first measured pose."
        )
        self.detail = [f"{len(steps)} human samples", f"Capture ended by {source['ended_by']}"]

    def replay(self):
        self.stage = "home"
        self.status = "Moving the mock arm to the recorded start pose…"
        if self.args.preview:
            self.begin("preview-home", 0.8)
            return
        if (self.draft / "no_human").exists():
            shutil.rmtree(self.draft / "no_human")
        if self.args.observe_replay:
            self.observed_at = self.finished_at = None
            self.start_bag()
            return
        replay_type = runpy.run_path(str(ROOT / "test-policy.py"))["Replay"]
        recording = self.draft / "human.json"
        playback = replay_type(recording)
        self.replay_steps = len(playback.deltas)
        if not 1 <= self.replay_steps <= 1200:
            raise ValueError(f"Mock replay needs 1–1,200 steps; this take expands to {self.replay_steps}.")
        write_json(
            self.draft / "replay-plan.json",
            {
                "start_joint_pos": playback.start,
                "actions": [playback.delta(step) for step in range(self.replay_steps)],
            },
        )
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", 0))
            port = sock.getsockname()[1]
        self.bag_topics.clear()
        self.start_job(
            "policy",
            ["mise", "run", "test-policy", "--", "--port", str(port), "--replay", str(recording)],
        )
        self.start_job(
            "runner",
            [
                "mise", "run", "policy-runner", "--",
                "--mode", "mock", "--task", self.task, "--task-pack", str(self.args.task_pack),
                "--policy", f"http://127.0.0.1:{port}", "--mock-replay-stdio",
                "--mock-replay-steps", str(self.replay_steps),
                "--mock-replay-start", *map(str, playback.start),
            ],
            domain=MOCK_DOMAIN, stdin=True,
        )
        self.begin("runner-start", 600)
        self.status = "Starting the isolated mock runner…"

    def start_bag(self):
        self.stage = "replay"
        self.status = "Waiting for the no-human recorder to subscribe…"
        self.bag_topics.clear()
        command = ["bash", "tools/record-three-cameras.sh"]
        if not self.args.observe_replay:
            command.append("--mock-state-only")
        self.start_job(
            "bag",
            [*command, str(self.draft / "no_human")],
            domain=CAPTURE_DOMAIN if self.args.observe_replay else MOCK_DOMAIN,
        )
        self.begin("bag-start", 35)

    def required_topics(self):
        return BAG_TOPICS | CAMERA_TOPICS if self.args.observe_replay else BAG_TOPICS

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
        elif name == "runner":
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
            if self.required_topics() <= self.bag_topics:
                if self.args.observe_replay:
                    self.begin("observing")
                    self.status = "Ready. Press B when the cell operator starts the existing replay."
                else:
                    self.jobs["runner"].send("run")
                    self.begin("replaying", 100)
                    self.status = "Replaying and capturing mock joint and controller states…"

    def finish_replay(self):
        self.begin(None)
        self.stage = "review"
        self.status = "Both takes are staged. Save or discard the pair."
        replay = (
            "sample data" if self.args.preview else
            "three-camera physical replay capture" if self.args.observe_replay else
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
                outputs = list((self.draft / "capture" / self.task).glob("*.json"))
                if len(outputs) != 1:
                    raise RuntimeError("The recorder did not produce one staged JSON file.")
                outputs[0].rename(self.draft / "human.json")
                shutil.rmtree(self.draft / "capture")
                self.finish_capture()
                self.stop_jobs()
                return
            if name == "runner" and self.operation == "bag-close" and finished == 0:
                continue
            if name == "bag" and self.operation == "bag-close":
                self.verify_bag()
                self.stop_jobs()
                self.finish_replay()
                return
            raise RuntimeError(f"{name.title()} exited with status {finished}. See logs/{name}.log.")
        if self.deadline is not None and monotonic() >= self.deadline:
            pending = sorted(self.required_topics() - self.bag_topics) if self.operation == "bag-start" else []
            suffix = f": {', '.join(pending)}" if pending else ""
            raise RuntimeError(f"Timed out during {self.operation}{suffix}.")

    def verify_bag(self):
        bag = self.draft / "no_human"
        metadata = bag / "metadata.yaml"
        if not metadata.is_file() or not any(bag.glob("*.mcap")):
            raise RuntimeError("The replay did not close with MCAP data and ROS bag metadata.")
        if self.args.observe_replay:
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
        if self.operation == "preview-home":
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
        elif self.args.observe_replay:
            replay = {
                "path": "no_human",
                "source": "operator_observed_physical_replay",
                "domain": int(CAPTURE_DOMAIN),
                "marked_start": self.observed_at,
                "marked_complete": self.finished_at,
                "camera_capture": True,
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
            self.status = "Capture failed. Start a new take when ready."
        elif self.stage in {"home", "replay"}:
            self.stage = "home"
            self.status = "The hand-guided take is still staged. Retry the no-human take or discard."

    def stop_jobs(self):
        for name in ("runner", "bag", "capture", "policy"):
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
    parser.add_argument(
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
    if args.preview and (args.input or args.observe_replay):
        parser.error("--preview uses sample data and cannot be combined with --input or --observe-replay.")
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
