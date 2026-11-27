#!/usr/bin/env python3
"""Run the three-view adapter and policy against an isolated HTTP physics engine."""
import argparse
import base64
import hashlib
import json
import math
import sys
import time
from pathlib import Path
from urllib.request import Request, urlopen


def http(base, route, body=None, timeout=60):
    request = Request(
        base + route,
        data=None if body is None else json.dumps(body, allow_nan=False).encode(),
        headers={"Content-Type": "application/json"},
    )
    with urlopen(request, timeout=timeout) as response:
        return json.load(response)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--adapter-dir", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--steps", type=int, default=300)
    args = parser.parse_args()
    if not 1 <= args.steps <= 1200:
        parser.error("--steps must be between 1 and 1200")
    sys.path.insert(0, str(args.adapter_dir))
    from three_view_proxy import Frames, TaskContext, translate

    # These are deliberately separate from the real-arm policy/proxy endpoints.
    engine = "http://127.0.0.1:18090"
    policy = "http://127.0.0.1:18082"
    for name, endpoint in [("simulation engine", engine), ("simulation policy", policy)]:
        if not http(endpoint, "/readyz").get("ready"):
            raise RuntimeError(f"{name} is not ready")
    args.out.mkdir(parents=True, exist_ok=False)
    state = http(engine, "/reset", {"task_id": "bolt_to_tray", "seed": 0})
    episode_id = state["episode_id"]
    obs = state["observation"]
    (args.out / "instruction.txt").write_text(obs["instruction"])
    goal = base64.b64decode(obs["goal"])
    (args.out / "goal.jpg").write_bytes(goal)
    context = TaskContext("bolt_to_tray", args.out / "instruction.txt", args.out / "goal.jpg")
    frames = Frames(0.1)
    initial = obs["proprio"]["joint_pos"]
    largest_change = 0.0
    nonzero_actions = 0
    completed = 0
    started = time.monotonic()
    with (args.out / "steps.jsonl").open("w") as journal:
        for index in range(args.steps):
            for view in ("wrist", "left", "right"):
                frames.update(view, base64.b64decode(obs["images"][view]))
            request = {
                "episode_id": episode_id,
                "task_id": "bolt_to_tray",
                "schema_version": obs["schema_version"],
                "step": state["step"],
                "images": {"top": obs["images"]["right"]},
                "proprio": obs["proprio"],
            }
            translated = json.loads(translate(json.dumps(request).encode(), frames, context))
            assert set(translated["images"]) == {"wrist", "left", "right"}
            assert translated["instruction"] == obs["instruction"]
            assert translated["goal"] == obs["goal"]
            if index == 0:
                (args.out / "first-request.json").write_text(json.dumps(translated))
            response = http(policy, "/act", translated)
            assert response["episode_id"] == episode_id
            assert response["step"] == state["step"]
            action = response["action"]
            assert action["type"] == "joint_delta"
            assert len(action["values"]) == 6
            assert all(math.isfinite(v) and abs(v) <= 0.05 for v in action["values"])
            assert 0 <= action["gripper"] <= 1
            nonzero_actions += any(abs(v) > 1e-6 for v in action["values"])
            previous_step = state["step"]
            state = http(engine, "/step", {
                "episode_id": episode_id,
                "step": previous_step,
                "action": action,
            })
            assert state["step"] > previous_step
            obs = state["observation"]
            largest_change = max(largest_change, max(
                abs(a - b) for a, b in zip(initial, obs["proprio"]["joint_pos"])
            ))
            journal.write(json.dumps({
                "step": state["step"], "action": action,
                "joint_pos": obs["proprio"]["joint_pos"],
                "done": state["done"], "success": state.get("success"),
            }) + "\n")
            journal.flush()
            completed += 1
            if completed % 25 == 0 or state["done"]:
                print(f"sim step {completed}: done={state['done']} success={state.get('success')}", flush=True)
            if state["done"]:
                break
    for view, data in obs["images"].items():
        (args.out / f"last-{view}.jpg").write_bytes(base64.b64decode(data))
    report = {
        "task": "bolt_to_tray", "seed": 0, "episode_id": episode_id,
        "steps": completed, "done": state["done"], "success": state.get("success"),
        "nonzero_actions": nonzero_actions,
        "largest_joint_change_rad": largest_change,
        "translation": "three_view_proxy.translate with simulation instruction and goal",
        "goal_sha256": hashlib.sha256(goal).hexdigest(),
        "elapsed_s": round(time.monotonic() - started, 3),
        "scope": "policy, translation function and MuJoCo physics; no ROS or physical-arm commands",
    }
    (args.out / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
