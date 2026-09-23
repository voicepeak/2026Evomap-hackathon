#!/bin/bash
# tablet_teleop · M0 一键启动
# 用法：  bash 启动.sh          （只开网页）
#        bash 启动.sh --bridge  （同时开 Python 桥）

set -u
cd "$(dirname "$0")"
PORT=8000

# 清掉占用端口的旧进程
lsof -ti tcp:$PORT 2>/dev/null | xargs kill -9 2>/dev/null
sleep 0.3

echo "→ 启动本地服务 http://127.0.0.1:$PORT"
python3 -m http.server $PORT --bind 127.0.0.1 >/dev/null 2>&1 &
HTTP_PID=$!

cleanup() {
  echo ""
  echo "→ 停止"
  kill $HTTP_PID 2>/dev/null
  [ -n "${BR_PID:-}" ] && kill $BR_PID 2>/dev/null
  exit 0
}
trap cleanup INT TERM

sleep 1
open "http://127.0.0.1:$PORT" 2>/dev/null || \
  echo "   （自己打开浏览器输入 http://127.0.0.1:$PORT）"

if [ "${1:-}" = "--bridge" ]; then
  if python3 -c "import websockets" 2>/dev/null; then
    echo "→ 启动 bridge ws://127.0.0.1:8765"
    python3 bridge.py &
    BR_PID=$!
  else
    echo "⚠️  没装 websockets，跳过 bridge。要装：pip install websockets"
  fi
fi

echo ""
echo "============================================================"
echo "  在浏览器里用笔划一下"
echo "  先看这四个：pointerType=pen / 采样率≈200 / 合并>1 / alt,az 有没有值"
echo "  Ctrl+C 停止"
echo "============================================================"
wait
