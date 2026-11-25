"""A stand-in policy server for the `/act` contract v0 (see API.md), standard library only.

It swings the chosen joints sinusoidally about the start pose, by default only the wrists by
±0.1 rad over 5 s, opening and closing the gripper every 5 s. With --offset it instead moves the palm
by an offset in `base_link` and back, keeping its orientation, with the gripper open on the way
there and closing on arrival to carry what it picked back. With --replay it plays back a
recorded episode's joint motion and gripper commands, open loop from wherever the arm starts. It
can be told to reply late, in chunks, or invalidly, to exercise the runner.
"""

import argparse
import base64
import json
import math
import threading
import time
from collections.abc import Callable
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, override

Reply = dict[str, Any]

PERIOD_STEPS = 100
STEP_S = 0.05
MAX_DELTA = 0.05
# The largest delta of a sine sampled PERIOD_STEPS times per cycle is 2·sin(π/PERIOD_STEPS) times
# its amplitude, and every delta must stay within the contract's MAX_DELTA.
MAX_AMPLITUDE = MAX_DELTA / (2 * math.sin(math.pi / PERIOD_STEPS))

# Each corrupts an otherwise valid reply, or its headers.
INVALID_KINDS: dict[str, Callable[[Reply, dict[str, str]], object]] = {
    "delta-out-of-range": lambda r, _: r["action"]["values"].__setitem__(0, 0.06),
    "gripper-out-of-range": lambda r, _: r["action"].__setitem__("gripper", 1.5),
    "five-values": lambda r, _: r["action"]["values"].pop(),
    "missing-gripper": lambda r, _: r["action"].pop("gripper"),
    "string-value": lambda r, _: r["action"]["values"].__setitem__(0, "0.0"),
    "unknown-field": lambda r, _: r["action"].__setitem__("speed", 1.0),
    "unknown-type": lambda r, _: r["action"].__setitem__("type", "cartesian"),
    "empty-chunk": lambda r, _: r.__setitem__("action", {"type": "action_chunk", "actions": []}),
    "long-chunk": lambda r, _: r.__setitem__(
        "action", {"type": "action_chunk", "actions": [r["action"]] * 17}
    ),
    "wrong-chunk-len": lambda r, _: r.__setitem__(
        "action", {"type": "action_chunk", "chunk_len": 2, "actions": [r["action"]]}
    ),
    "wrong-episode-id": lambda r, _: r.__setitem__("episode_id", "other"),
    "wrong-step": lambda r, _: r.__setitem__("step", r["step"] + 1),
    "wrong-step-header": lambda _, h: h.__setitem__("X-RC-Step", "-1"),
    "missing-episode-header": lambda _, h: h.pop("X-RC-Episode-Id"),
    "http-500": lambda _, h: h.__setitem__("status", "500"),
}


def offset(step: int, amplitude: float) -> float:
    return amplitude * math.sin(2 * math.pi * step / PERIOD_STEPS)


def joint_delta(step: int, amplitude: float, joints: frozenset[int]) -> Reply:
    """Moves the joints from their offset at `step` to the one at `step + 1`, so the arm swings
    about the start pose instead of drifting from it."""
    delta = round(offset(step + 1, amplitude) - offset(step, amplitude), 6)
    return {
        "type": "joint_delta",
        "values": [delta if joint in joints else 0.0 for joint in range(6)],
        "gripper": 1.0 if (step // PERIOD_STEPS) % 2 == 0 else 0.0,
    }


Matrix = list[list[float]]
Joints = list[float]

# (d, a, alpha) per joint, as the runner's src/kinematics.rs, so that poses agree with its ee_pos.
UR12E_DH = [
    (0.1807, 0.0, math.pi / 2),
    (0.0, -0.6127, 0.0),
    (0.0, -0.57155, 0.0),
    (0.17415, 0.0, math.pi / 2),
    (0.11985, 0.0, -math.pi / 2),
    (0.11655, 0.0, 0.0),
]
BASE_LINK_TO_BASE: Matrix = [[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
TOOL0_TO_PALM: Matrix = [[-1, 0, 0, 0], [0, 0, 1, 0.107], [0, 1, 0, 0], [0, 0, 0, 1]]

GO_STEPS = 120
HOLD_STEPS = 60
IK_TOLERANCE = 1e-7
IK_DAMPING = 1e-3


def matmul(a: Matrix, b: Matrix) -> Matrix:
    return [
        [sum(a[i][k] * b[k][j] for k in range(len(b))) for j in range(len(b[0]))]
        for i in range(len(a))
    ]


def dh(d: float, a: float, alpha: float, theta: float) -> Matrix:
    ct, st, ca, sa = math.cos(theta), math.sin(theta), math.cos(alpha), math.sin(alpha)
    return [
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0],
    ]


def palm_pose(q: Joints) -> Matrix:
    pose = BASE_LINK_TO_BASE
    for (d, a, alpha), theta in zip(UR12E_DH, q, strict=True):
        pose = matmul(pose, dh(d, a, alpha, theta))
    return matmul(pose, TOOL0_TO_PALM)


def cross(u: list[float], v: list[float]) -> list[float]:
    return [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]


def pose_error(target: Matrix, actual: Matrix) -> list[float]:
    """Position error, then the small-angle rotation taking `actual`'s axes onto `target`'s."""
    position = [target[i][3] - actual[i][3] for i in range(3)]
    rotation = [0.0, 0.0, 0.0]
    for axis in range(3):
        turn = cross([actual[i][axis] for i in range(3)], [target[i][axis] for i in range(3)])
        rotation = [r + 0.5 * t for r, t in zip(rotation, turn, strict=True)]
    return position + rotation


def solve(a: Matrix, b: list[float]) -> list[float]:
    """Gaussian elimination with partial pivoting."""
    n = len(b)
    m = [[*row, rhs] for row, rhs in zip(a, b, strict=True)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(m[r][col]))
        m[col], m[pivot] = m[pivot], m[col]
        for r in range(col + 1, n):
            factor = m[r][col] / m[col][col]
            m[r] = [x - factor * y for x, y in zip(m[r], m[col], strict=True)]
    x = [0.0] * n
    for r in reversed(range(n)):
        x[r] = (m[r][n] - sum(m[r][c] * x[c] for c in range(r + 1, n))) / m[r][r]
    return x


def inverse_kinematics(target: Matrix, seed: Joints) -> tuple[Joints, float]:
    """Damped least squares from `seed`; returns the joints and the remaining error's norm."""
    q = list(seed)
    error = math.inf
    for _ in range(50):
        actual = palm_pose(q)
        e = pose_error(target, actual)
        error = math.sqrt(sum(x * x for x in e))
        if error < IK_TOLERANCE:
            break
        h = 1e-7
        columns = []
        for joint in range(6):
            nudged = list(q)
            nudged[joint] += h
            moved = pose_error(palm_pose(nudged), actual)
            columns.append([x / h for x in moved])
        jacobian = [[columns[j][i] for j in range(6)] for i in range(6)]
        jjt = [
            [
                sum(jacobian[i][k] * jacobian[j][k] for k in range(6))
                + (IK_DAMPING if i == j else 0.0)
                for j in range(6)
            ]
            for i in range(6)
        ]
        y = solve(jjt, e)
        q = [qi + sum(jacobian[k][i] * y[k] for k in range(6)) for i, qi in enumerate(q)]
    return q, error


def progress(step: int) -> float:
    """0 to 1 over GO_STEPS, held, then back to 0 over GO_STEPS, easing in and out."""
    if step <= GO_STEPS:
        u = step / GO_STEPS
    elif step <= GO_STEPS + HOLD_STEPS:
        u = 1.0
    else:
        u = max(0.0, 1.0 - (step - GO_STEPS - HOLD_STEPS) / GO_STEPS)
    return 0.5 - 0.5 * math.cos(math.pi * u)


class PalmPath:
    """The joint positions that move the palm by `offset` and back, solved one step at a time from
    the joints the episode started at, so that the deltas between them add up to the path."""

    def __init__(self, start: Joints, offset: list[float]) -> None:
        self.start_pose: Matrix = palm_pose(start)
        self.offset: list[float] = offset
        self.joints: list[Joints] = [list(start)]
        self.failed_at: int | None = None

    def target(self, step: int) -> Matrix:
        pose = [list(row) for row in self.start_pose]
        for i in range(3):
            pose[i][3] += self.offset[i] * progress(step)
        return pose

    def at(self, step: int) -> Joints:
        while len(self.joints) <= step and self.failed_at is None:
            index = len(self.joints)
            q, error = inverse_kinematics(self.target(index), self.joints[-1])
            jump = max(abs(a - b) for a, b in zip(q, self.joints[-1], strict=True))
            if error > 1e-4 or jump > MAX_DELTA:
                self.failed_at = index
                print(
                    f"Step {index}: no smooth solution (error {error:.2e}, jump {jump:.3f} rad); holding."
                )
                break
            self.joints.append(q)
        return self.joints[min(step, len(self.joints) - 1)]

    def delta(self, step: int) -> Reply:
        before, after = self.at(step), self.at(step + 1)
        return {
            "type": "joint_delta",
            "values": [round(b - a, 6) for a, b in zip(before, after, strict=True)],
            # Closes on arrival, so the gripper has the hold to finish before the way back.
            "gripper": 0.0 if step >= GO_STEPS else 1.0,
        }


class Replay:
    """The joint deltas and gripper commands of an episode saved by `mise run record`, then holding
    still once the recording runs out."""

    def __init__(self, path: Path) -> None:
        steps = json.loads(path.read_text())["steps"]
        self.start: Joints = steps[0]["proprio"]["joint_pos"]
        self.grippers: list[float] = [step["gripper_command"] for step in steps]
        self.deltas: list[Joints] = []
        # A step the arm moved more than MAX_DELTA in is spread over the following ones, so that
        # the replay still ends where the recording did.
        commanded = list(self.start)
        for step in steps[1:]:
            self.deltas.append(self.step_towards(commanded, step["proprio"]["joint_pos"]))
        self.recorded_steps: int = len(self.deltas)
        end = steps[-1]["proprio"]["joint_pos"]
        while any(abs(e - c) > 1e-6 for e, c in zip(end, commanded, strict=True)):
            self.deltas.append(self.step_towards(commanded, end))

    @staticmethod
    def step_towards(commanded: Joints, target: Joints) -> Joints:
        delta = [
            round(max(-MAX_DELTA, min(MAX_DELTA, t - c)), 6)
            for t, c in zip(target, commanded, strict=True)
        ]
        commanded[:] = [c + d for c, d in zip(commanded, delta, strict=True)]
        return delta

    def delta(self, step: int) -> Reply:
        if step >= len(self.deltas):
            return {"type": "joint_delta", "values": [0.0] * 6, "gripper": self.grippers[-1]}
        return {
            "type": "joint_delta",
            "values": self.deltas[step],
            "gripper": self.grippers[min(step + 1, len(self.grippers) - 1)],
        }


def offset_arg(text: str) -> list[float]:
    values = [float(v) for v in text.split(",")]
    if len(values) != 3:
        raise argparse.ArgumentTypeError("expected X,Y,Z in metres")
    return values


def amplitude_arg(text: str) -> float:
    value = float(text)
    if not 0 <= value <= MAX_AMPLITUDE:
        raise argparse.ArgumentTypeError(f"must be within [0, {MAX_AMPLITUDE:.3f}] rad")
    return value


def joints_arg(text: str) -> frozenset[int]:
    joints = frozenset(int(joint) for joint in text.split(",") if joint)
    if not joints <= set(range(6)):
        raise argparse.ArgumentTypeError("joint indices must be within 0 to 5")
    return joints


class Policy:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args: argparse.Namespace = args
        self.ready_at: float = time.monotonic() + args.load_time
        self.seen: set[tuple[str, int]] = set()
        self.lock: threading.Lock = threading.Lock()
        self.paths: dict[str, PalmPath] = {}
        self.replay: Replay | None = None
        self.replayed_episode: str | None = None
        if args.replay is not None:
            self.replay = Replay(args.replay)
            recorded, total = self.replay.recorded_steps, len(self.replay.deltas)
            print(
                f"Replaying {args.replay}: {recorded} steps, {total * STEP_S:.2f} s with catch-up"
            )

    def ready(self) -> bool:
        return time.monotonic() >= self.ready_at

    def replay_delta(self, replay: Replay, request: Reply, step: int) -> Reply:
        episode_id = request["episode_id"]
        with self.lock:
            if self.replayed_episode != episode_id:
                self.replayed_episode = episode_id
                mismatch = max(
                    abs(a - b)
                    for a, b in zip(request["proprio"]["joint_pos"], replay.start, strict=True)
                )
                print(
                    f"Episode {episode_id}: starting {mismatch:.4f} rad from the recording's start"
                )
        return replay.delta(step)

    def joint_delta(self, request: Reply, step: int) -> Reply:
        if self.replay is not None:
            return self.replay_delta(self.replay, request, step)
        if self.args.offset is None:
            return joint_delta(step, self.args.amplitude, self.args.joints)
        episode_id = request["episode_id"]
        with self.lock:
            path = self.paths.get(episode_id)
            if path is None:
                path = PalmPath(request["proprio"]["joint_pos"], self.args.offset)
                self.paths = {episode_id: path}
                start = [row[3] for row in path.start_pose[:3]]
                mismatch = max(
                    abs(a - b) for a, b in zip(start, request["proprio"]["ee_pos"], strict=True)
                )
                print(
                    f"Episode {episode_id}: palm at {[round(x, 4) for x in start]}, {mismatch * 1000:.2f} mm from ee_pos"
                )
            return path.delta(step)

    def act(self, request: Reply) -> tuple[Reply, dict[str, str]]:
        episode_id, step = request["episode_id"], request["step"]
        if self.args.log_requests:
            print(f"{time.monotonic():.3f} {episode_id} {step} {request['task_id']}", flush=True)
        with self.lock:
            first_attempt = (episode_id, step) not in self.seen
            self.seen.add((episode_id, step))

        if self.args.hang:
            time.sleep(3600)
        delay = self.args.delay
        if self.args.slow_every and step % self.args.slow_every == 0 and first_attempt:
            delay = self.args.slow_delay
        time.sleep(delay / 1000)

        if self.args.save_image:
            Path(self.args.save_image).write_bytes(base64.b64decode(request["images"]["top"]))

        if self.args.chunk > 1:
            actions = [self.joint_delta(request, step + i) for i in range(self.args.chunk)]
            action = {"type": "action_chunk", "chunk_len": len(actions), "actions": actions}
        else:
            action = self.joint_delta(request, step)
        reply: Reply = {"episode_id": episode_id, "step": step, "action": action}
        headers = {"X-RC-Episode-Id": episode_id, "X-RC-Step": str(step)}
        if self.args.invalid and step >= self.args.invalid_from:
            INVALID_KINDS[self.args.invalid](reply, headers)
        return reply, headers


class Handler(BaseHTTPRequestHandler):
    # Keeps connections alive between requests, as the runner's are.
    protocol_version: str = "HTTP/1.1"
    # Headers and body go out in separate writes, so with Nagle's algorithm the body would wait
    # ~40 ms for the client's delayed ACK.
    disable_nagle_algorithm = True
    policy: Policy

    def do_GET(self) -> None:
        if self.path == "/healthz":
            self.reply(200, {})
        elif self.path == "/readyz":
            ready = self.policy.ready()
            args = self.policy.args
            self.reply(
                200,
                {"ready": ready, "contract": args.contract, "schema_version": args.schema_version},
            )
        else:
            self.reply(404, {})

    def do_POST(self) -> None:
        body = self.rfile.read(int(self.headers.get("Content-Length", 0)))
        if self.path != "/act":
            self.reply(404, {})
            return
        reply, headers = self.policy.act(json.loads(body))
        status = int(headers.pop("status", "200"))
        self.reply(status, reply, headers)

    def reply(self, status: int, body: Reply, headers: dict[str, str] | None = None) -> None:
        data = json.dumps(body).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        for name, value in (headers or {}).items():
            self.send_header(name, value)
        self.end_headers()
        _ = self.wfile.write(data)

    @override
    def log_message(self, format: str, *args: Any) -> None:
        pass


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    _ = parser.add_argument("--port", type=int, default=8000)
    _ = parser.add_argument("--load-time", type=float, default=0.0, help="seconds until ready")
    _ = parser.add_argument("--contract", default="v0", help="the contract /readyz reports")
    _ = parser.add_argument(
        "--schema-version", type=int, default=2, help="the schema version /readyz reports"
    )
    _ = parser.add_argument(
        "--amplitude",
        type=amplitude_arg,
        default=0.1,
        help=f"rad each moving joint swings about the start pose, at most {MAX_AMPLITUDE:.3f}",
    )
    _ = parser.add_argument(
        "--joints",
        type=joints_arg,
        default=frozenset({3, 4, 5}),
        help="comma-separated indices of the joints that move, 0 shoulder pan to 5 wrist 3",
    )
    _ = parser.add_argument(
        "--offset",
        type=offset_arg,
        help="X,Y,Z metres in base_link: move the palm by this and back instead of swinging joints",
    )
    _ = parser.add_argument(
        "--replay",
        type=Path,
        help="an episode from recording/: replay its motion and gripper commands instead",
    )
    _ = parser.add_argument("--delay", type=float, default=5.0, help="ms before every reply")
    _ = parser.add_argument(
        "--slow-every", type=int, default=0, help="delay the first reply to every N-th step"
    )
    _ = parser.add_argument("--slow-delay", type=float, default=150.0, help="ms, with --slow-every")
    _ = parser.add_argument("--chunk", type=int, default=1, help="actions per reply")
    _ = parser.add_argument("--invalid", choices=sorted(INVALID_KINDS), help="corrupt every reply")
    _ = parser.add_argument(
        "--invalid-from", type=int, default=0, help="the first step to corrupt, with --invalid"
    )
    _ = parser.add_argument("--hang", action="store_true", help="never reply to /act")
    _ = parser.add_argument("--log-requests", action="store_true", help="print each /act call")
    _ = parser.add_argument("--save-image", help="write the last received image to this file")
    args = parser.parse_args()
    if args.replay is not None and args.offset is not None:
        parser.error("--replay and --offset are alternatives")

    Handler.policy = Policy(args)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    server.daemon_threads = True
    print(f"Stand-in policy on http://127.0.0.1:{args.port}")
    server.serve_forever()


if __name__ == "__main__":
    main()
