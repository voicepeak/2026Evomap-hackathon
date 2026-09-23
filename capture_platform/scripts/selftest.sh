#!/usr/bin/env bash
# 无硬件自检：遥操 → 录制 → 质检 → 打包
set -euo pipefail
cd "$(dirname "$0")/.."

if [ -x ".venv/bin/rebot-capture" ]; then
  BIN=".venv/bin/rebot-capture"
else
  BIN="rebot-capture"
fi
exec "$BIN" selftest
