#!/usr/bin/env bash
# 启动采集端本地服务（默认 Mock 后端）
set -euo pipefail
cd "$(dirname "$0")/.."

if [ -x ".venv/bin/rebot-capture" ]; then
  BIN=".venv/bin/rebot-capture"
else
  BIN="rebot-capture"
fi

echo "→ 启动 rebot-capture（backend=mock）: http://127.0.0.1:8787"
"$BIN" serve --backend mock --host 127.0.0.1 --port 8787
