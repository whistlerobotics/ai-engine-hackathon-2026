#!/usr/bin/env python3
"""Experimental ROS 2 three-camera adapter for the Reality Check policy API."""
import argparse
import base64
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen

VIEWS = ("wrist", "left", "right")
SIZE = (640, 360)


def jpeg_size(data):
    """Read a JPEG's SOF dimensions; reject malformed headers and non-JPEGs."""
    if data[:2] != b"\xff\xd8":
        raise ValueError("not a JPEG")
    i = 2
    while i < len(data):
        if data[i] != 0xff:
            raise ValueError("invalid JPEG marker")
        while i < len(data) and data[i] == 0xff:
            i += 1
        if i >= len(data):
            break
        marker = data[i]
        i += 1
        if marker in (0xd8, 0xd9, 0x01) or 0xd0 <= marker <= 0xd7:
            continue
        if i + 2 > len(data):
            break
        length = int.from_bytes(data[i:i+2], "big")
        if length < 2 or i + length > len(data):
            break
        if marker in (0xc0, 0xc1, 0xc2, 0xc3, 0xc5, 0xc6, 0xc7, 0xc9, 0xca, 0xcb, 0xcd, 0xce, 0xcf):
            if length < 8:
                break
            return int.from_bytes(data[i+5:i+7], "big"), int.from_bytes(data[i+3:i+5], "big")
        if marker == 0xda:
            break
        i += length
    raise ValueError("JPEG has no valid frame header")


class Frames:
    def __init__(self, max_age):
        self.max_age = max_age
        self.lock = threading.Lock()
        self.frames = {}

    def update(self, view, data):
        if view not in VIEWS:
            raise ValueError("unknown view")
        try:
            if jpeg_size(data) != SIZE:
                raise ValueError("expected a 640x360 JPEG")
        except ValueError:
            with self.lock:
                self.frames.pop(view, None)
            raise
        encoded = base64.b64encode(data).decode("ascii")
        with self.lock:
            self.frames[view] = (time.monotonic(), encoded)

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            for view in VIEWS:
                if view not in self.frames or now - self.frames[view][0] > self.max_age:
                    raise ValueError(f"{view} camera missing, invalid or stale")
            return {view: self.frames[view][1] for view in VIEWS}


class TaskContext:
    def __init__(self, task_id, instruction_path, goal_path):
        self.task_id = task_id
        self.instruction = Path(instruction_path).read_text().strip()
        if not self.instruction:
            raise ValueError("task instruction is empty")
        goal = Path(goal_path).read_bytes()
        self.goal_size = jpeg_size(goal)
        self.goal = base64.b64encode(goal).decode("ascii")

    def apply(self, request):
        if request.get("task_id") != self.task_id:
            raise ValueError("request task_id does not match configured task context")
        request["instruction"] = self.instruction
        request["goal"] = self.goal


def translate(body, frames, task_context=None):
    request = json.loads(body)
    if not isinstance(request, dict) or not isinstance(request.get("images"), dict):
        raise ValueError("request must contain images")
    request["images"] = frames.snapshot()
    if task_context is not None:
        task_context.apply(request)
    return json.dumps(request, allow_nan=False, separators=(",", ":")).encode()


def create_server(host, port, upstream, frames, task_context=None, upstream_timeout=1):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def do_GET(self):
            self.forward()

        def do_POST(self):
            self.forward()

        def forward(self):
            try:
                if (self.command, self.path) not in (("GET", "/healthz"), ("GET", "/readyz"), ("POST", "/act")):
                    self.send_error(404)
                    return
                body = None
                if self.command == "POST":
                    n = int(self.headers.get("Content-Length", "0"))
                    if not 0 < n <= 16 * 1024 * 1024:
                        raise ValueError("invalid content length")
                    body = translate(self.rfile.read(n), frames, task_context)
                headers = {k: v for k, v in self.headers.items() if k.lower() not in
                           ("host", "content-length", "connection", "transfer-encoding")}
                req = Request(upstream.rstrip("/") + self.path, data=body, headers=headers, method=self.command)
                try:
                    response = urlopen(req, timeout=upstream_timeout)
                except HTTPError as exc:
                    response = exc
                with response:
                    data = response.read()
                    self.send_response(response.status)
                    for key, value in response.headers.items():
                        if key.lower() not in ("content-length", "connection", "transfer-encoding", "server", "date"):
                            self.send_header(key, value)
                    self.send_header("Content-Length", str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
            except (ValueError, TypeError, json.JSONDecodeError) as exc:
                self.send_error(503, str(exc))
            except (URLError, OSError, TimeoutError):
                self.send_error(502, "policy upstream unavailable")
    return ThreadingHTTPServer((host, port), Handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--upstream", required=True, help="policy server URL")
    for view in VIEWS:
        parser.add_argument(f"--{view}-topic", required=True, help="ROS CompressedImage topic")
    parser.add_argument("--listen", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8001)
    parser.add_argument("--max-age", type=float, default=0.1, help="maximum frame age in seconds")
    parser.add_argument("--task", help="task_id for the configured instruction and goal")
    parser.add_argument("--instruction-file", type=Path, help="task instruction text file")
    parser.add_argument("--goal-image", type=Path, help="goal JPEG; native dimensions are preserved")
    parser.add_argument("--upstream-timeout", type=float, default=1,
                        help="policy request timeout in seconds; allow longer for initial VLM planning")
    args = parser.parse_args()
    url = urlsplit(args.upstream)
    if url.scheme not in ("http", "https") or not url.netloc:
        parser.error("--upstream must be an http(s) URL")
    if not 0 < args.max_age <= 5:
        parser.error("--max-age must be > 0 and <= 5 seconds")
    topics = [getattr(args, f"{view}_topic") for view in VIEWS]
    if len(set(topics)) != 3:
        parser.error("each camera must use a distinct topic")
    if not 0 < args.upstream_timeout <= 120:
        parser.error("--upstream-timeout must be > 0 and <= 120 seconds")
    context_args = (args.task, args.instruction_file, args.goal_image)
    if any(context_args) and not all(context_args):
        parser.error("--task, --instruction-file and --goal-image must be supplied together")
    try:
        task_context = TaskContext(*context_args) if all(context_args) else None
    except (OSError, ValueError) as exc:
        parser.error(str(exc))

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage

    rclpy.init()
    node = rclpy.create_node("reality_check_three_view_proxy")
    frames = Frames(args.max_age)
    def callback(view):
        def receive(message):
            try:
                frames.update(view, bytes(message.data))
            except ValueError as exc:
                node.get_logger().error(f"{view}: {exc}", throttle_duration_sec=5.0)
        return receive
    subscriptions = [node.create_subscription(CompressedImage, topic, callback(view), qos_profile_sensor_data)
                     for view, topic in zip(VIEWS, topics)]
    server = create_server(args.listen, args.port, args.upstream, frames,
                           task_context, args.upstream_timeout)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        rclpy.shutdown()
        del subscriptions


if __name__ == "__main__":
    main()
