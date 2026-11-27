"""Loopback TLS peer for the Rust client tests. EOF on stdin shuts it down."""

import json
import ssl
import subprocess
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


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
            "action": {"type": "joint_delta", "values": [0.1] * 6, "gripper": 1},
        }
        try:
            self.reply(reply, [
                ("X-RC-Episode-Id", self.headers["X-RC-Episode-Id"]),
                ("X-RC-Step", self.headers["X-RC-Step"]),
            ])
        except OSError:
            pass


def openssl(root, *args):
    subprocess.run(["openssl", *args], check=True, cwd=root, stdout=subprocess.DEVNULL,
                   stderr=subprocess.PIPE)


with tempfile.TemporaryDirectory() as name:
    root = Path(name)
    for ca in ["ca", "untrusted"]:
        openssl(root, "req", "-x509", "-newkey", "rsa:2048", "-nodes", "-days", "1",
                "-subj", "/CN=" + ca, "-keyout", ca + ".key", "-out", ca + ".pem")
    openssl(root, "req", "-newkey", "rsa:2048", "-nodes", "-subj", "/CN=localhost",
            "-keyout", "server.key", "-out", "server.csr")
    (root / "index.txt").touch()
    (root / "serial").write_text("01\n")
    (root / "ca.conf").write_text(
        "[ca]\ndefault_ca=signer\n"
        "[signer]\ndatabase=index.txt\nnew_certs_dir=.\ncertificate=ca.pem\nprivate_key=ca.key\n"
        "serial=serial\ndefault_md=sha256\npolicy=names\nx509_extensions=server\n"
        "[names]\ncommonName=supplied\n"
        "[server]\nsubjectAltName=DNS:localhost\nextendedKeyUsage=serverAuth\n"
    )
    validity = (
        ["-startdate", "20000101000000Z", "-enddate", "20000102000000Z"]
        if sys.argv[1] == "-1"
        else ["-days", sys.argv[1]]
    )
    openssl(root, "ca", "-batch", "-notext", "-config", "ca.conf", *validity,
            "-in", "server.csr", "-out", "server.pem")
    with ThreadingHTTPServer(("127.0.0.1", 0), Handler) as server:
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.load_cert_chain(root / "server.pem", root / "server.key")
        server.socket = ctx.wrap_socket(server.socket, server_side=True)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        print(json.dumps([name, server.server_port]), flush=True)
        sys.stdin.readline()
        server.shutdown()
