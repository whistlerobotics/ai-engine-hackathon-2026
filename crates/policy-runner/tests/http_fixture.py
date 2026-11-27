"""Loopback HTTP peer for the Rust client tests. EOF on stdin shuts it down."""

import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


class Handler(BaseHTTPRequestHandler):
    calls = 0

    def log_message(self, *_):
        pass

    def reply(self, body, headers=()):
        data = json.dumps(body).encode()
        self.send_response(200)
        for key, value in headers:
            self.send_header(key, value)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == "/healthz":
            self.reply({"calls": Handler.calls})
        elif self.path == "/readyz":
            self.reply({"ready": True, "contract": "v0", "schema_version": 3})
        else:
            self.send_error(404)

    def do_POST(self):
        request = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
        Handler.calls += 1
        if request["task_id"] == "slow":
            time.sleep(0.5)
        reply = {
            "episode_id": request["episode_id"],
            "step": request["step"],
            "action": {"type": "joint_delta", "values": [0.04] * 6, "gripper": 1},
        }
        try:
            self.reply(reply, [
                ("X-RC-Episode-Id", self.headers["X-RC-Episode-Id"]),
                ("X-RC-Step", self.headers["X-RC-Step"]),
            ])
        except OSError:
            pass


with ThreadingHTTPServer(("127.0.0.1", 0), Handler) as server:
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(server.server_port, flush=True)
    sys.stdin.readline()
    server.shutdown()
