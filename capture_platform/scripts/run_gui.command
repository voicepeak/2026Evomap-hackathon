#!/bin/zsh
# 桌面 GUI 启动器：一次双击 = 拉起采集服务（继承 Terminal 相机权限）+ 打开原生界面。
# 服务已在 8787 运行时只开界面，不会起第二个控制进程。
#
# caffeinate：演示期间阻止显示器休眠（否则全屏画面会变黑，误以为程序挂了）。
set -eu
CAPTURE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
ARM_REPO="$(cd "$CAPTURE_DIR/../arm_control" && pwd)"
cd "$CAPTURE_DIR"
exec > >(tee -a /tmp/rebot_gui.log) 2>&1
printf '\n=== rebot GUI %s ===\n' "$(date '+%Y-%m-%d %H:%M:%S')"
exec caffeinate -d -i "$CAPTURE_DIR/.venv/bin/rebot-capture" gui \
  --url http://127.0.0.1:8787 \
  --backend rebot \
  --arm-repo "$ARM_REPO" \
  --gesture-camera 2 \
  --service-log /tmp/rebot_gui_serve.log
