#!/bin/zsh
# 在 Terminal 里启动平台服务（继承 Terminal 的相机权限）
# 用法：双击本文件，或 osascript 让 Terminal 执行它
cd "/Users/Admin/Documents/Default Project/rebot_capture" || exit 1
echo "=== rebot-capture serve 启动 $(date '+%H:%M:%S') ===" | tee /tmp/rebot_serve.log
exec .venv/bin/rebot-capture serve \
  --backend rebot \
  --arm-repo /Users/Admin/Desktop/reBotArm_control_py \
  --port 8787 2>&1 | tee -a /tmp/rebot_serve.log
