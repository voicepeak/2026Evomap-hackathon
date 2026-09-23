#!/bin/zsh
# 从当前工作区启动，继承 Terminal 的相机权限。
set -eu
CAPTURE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
ARM_REPO="$(cd "$CAPTURE_DIR/../arm_control" && pwd)"
cd "$CAPTURE_DIR"
exec > >(tee -a /tmp/rebot_workspace_serve.log) 2>&1
printf '\n=== rebot-capture serve %s ===\n' "$(date '+%Y-%m-%d %H:%M:%S')"
exec "$CAPTURE_DIR/.venv/bin/rebot-capture" serve \
  --backend rebot \
  --arm-repo "$ARM_REPO" \
  --gesture-camera 2 \
  --host 127.0.0.1 \
  --port 8787
