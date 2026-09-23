#!/bin/zsh
# 一键启动（没有机械臂时用）：mock 后端 —— 界面全功能可用，只是不驱动真机。
# 有机械臂时请用 run_gui.command。
set -eu
CAPTURE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$CAPTURE_DIR"
exec > >(tee -a /tmp/rebot_gui.log) 2>&1
printf '\n=== rebot GUI (mock) %s ===\n' "$(date '+%Y-%m-%d %H:%M:%S')"
exec caffeinate -d -i "$CAPTURE_DIR/.venv/bin/rebot-capture" gui \
  --url http://127.0.0.1:8787 \
  --backend mock \
  --gesture-camera 0 \
  --service-log /tmp/rebot_gui_serve.log
