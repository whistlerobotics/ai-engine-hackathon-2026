#!/usr/bin/env bash
set -euo pipefail
policy_code="$HOME/rc-qwen-visionik-policy"
policy_weights="$HOME/models/rc-qwen-visionik-policy"
for shard in 00001 00002 00003 00004; do
  test -f "$policy_weights/qwen3_vl_8b/model-$shard-of-00004.safetensors"
done
exec docker run --name rc-qwen-visionik --gpus all \
  --user 10001:10001 --cap-drop ALL --security-opt no-new-privileges \
  -p 127.0.0.1:18080:8080 --tmpfs /scratch:rw,size=8g,exec \
  -e TMPDIR=/scratch -e HF_HOME=/scratch/hf \
  -e HF_HUB_OFFLINE=1 -e TRANSFORMERS_OFFLINE=1 \
  -e RC_WEIGHTS_DIR=/weights -e RC_VLM_QUANT=bf16 \
  -v "$policy_code:/submission:ro" -v "$policy_weights:/weights:ro" \
  -w /submission --entrypoint python \
  ghcr.io/lucasucal/rc-policy@sha256:19e4140d1a132807ac2eb8140a3d4eba4fa130cd10aa8cfcbc6f827a96ffa588 serve.py
