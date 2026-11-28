#!/usr/bin/env bash
set -euo pipefail

repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
task=${1:-bolt_to_tray}
if [[ $# -gt 0 ]]; then shift; fi

exec bash "$repo_dir/tools/run-legacy-policy-runner.sh" \
    --task "$task" \
    --policy "${WHISTLE_POLICY_URL:-http://127.0.0.1:18081}" \
    --camera-topic /right_camera/color/image_raw/compressed \
    --camera-size 640x360 \
    "$@"
