#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/kilted/setup.bash
source /home/rosdev/robotiq_ws/install/local_setup.sh
set -u

repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
revision=d6f0ace748600d2a94dc09de5b7711a6d127018a
source_dir="$repo_dir/target/legacy-policy-runner/source-$revision"
target_dir="$repo_dir/target/legacy-policy-runner/build"
if [[ ! -f "$source_dir/Cargo.toml" ]]; then
    git -C "$repo_dir" cat-file -e "$revision^{commit}"
    mkdir -p "$source_dir"
    git -C "$repo_dir" archive "$revision" | tar -x -C "$source_dir"
fi

python3 - "$repo_dir/crates/policy-runner/src/worker.rs" "$source_dir/crates/policy-runner/src/worker.rs" <<'PY'
import ast
import math
from pathlib import Path
import re
import sys

active, legacy = map(Path, sys.argv[1:])
pattern = r"pub const START_POSE: Joints = (\[[^\]]+\]);"
matches = re.findall(pattern, active.read_text())
if len(matches) != 1:
    raise SystemExit("Cannot identify the active checkout's start pose")
pose = ast.literal_eval(matches[0])
if len(pose) != 6 or not all(math.isfinite(value) for value in pose):
    raise SystemExit("The start pose must contain six finite joint angles")
original = legacy.read_text()
updated, count = re.subn(pattern, f"pub const START_POSE: Joints = {matches[0]};", original)
if count != 1:
    raise SystemExit("Cannot identify the legacy runner's start pose")
updated = updated.replace(
    "/// Captured from the real arm on 2026-11-25; not the contract's example pose.",
    "/// Synchronized from the active checkout's captured start pose.",
)
if updated != original:
    legacy.write_text(updated)
    print("Updated legacy start pose from the active checkout.", file=sys.stderr)
PY

exec cargo run --locked --release \
    --manifest-path "$source_dir/Cargo.toml" \
    --target-dir "$target_dir" -p policy-runner -- "$@"
