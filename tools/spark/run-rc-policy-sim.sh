#!/usr/bin/env bash
set -euo pipefail

docker start rc-policy-sim-engine rc-policy-sim-model >/dev/null
python3 - <<'PY'
import json
import time
from urllib.request import urlopen

for port, name in [(18090, "simulation engine"), (18082, "simulation policy")]:
    deadline = time.monotonic() + 240
    while True:
        try:
            with urlopen(f"http://127.0.0.1:{port}/readyz", timeout=3) as response:
                if json.load(response).get("ready"):
                    print(f"{name}: ready", flush=True)
                    break
        except OSError:
            pass
        if time.monotonic() > deadline:
            raise SystemExit(f"{name} did not become ready")
        print(f"Waiting for {name}...", flush=True)
        time.sleep(5)
PY

exec python3 "$HOME/ai-engine-hackathon-2026/tools/test-policy-in-sim.py" \
    --adapter-dir "$HOME/ai-engine-hackathon-2026-pr7/tools" \
    --out "$HOME/ai-engine-hackathon-2026/recording/sim-bolt-to-tray-$(date -u +%Y%m%dT%H%M%SZ)-$$" \
    --steps "${1:-300}"
