#!/bin/zsh
# 在 Terminal 运行：停止本工作区旧服务，启动新版，恢复腕部画面（相机索引可改，
# 默认 REBOT_ARM_CAMERA=0 = 机械臂上的那路相机）。
set -eu
CAPTURE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$CAPTURE_DIR"
.venv/bin/python - <<'PY'
import json, os, signal, subprocess, time, urllib.request
from pathlib import Path
from rebot_capture.processes import wait_for_exit
root = Path.cwd()
r = subprocess.run(['lsof', '-t', '-iTCP:8787', '-sTCP:LISTEN'], capture_output=True, text=True)
pids = set(r.stdout.split())
for text in pids:
    pid = int(text)
    command = subprocess.check_output(['ps', '-p', str(pid), '-o', 'command='], text=True)
    owned = (str(root / '.venv/bin/rebot-capture') in command
             or (str(root / '.venv/bin/python') in command and 'rebot-capture serve ' in command))
    if not owned:
        raise SystemExit('8787 被其他目录的服务占用，请先关闭该服务。')
    with urllib.request.urlopen('http://127.0.0.1:8787/api/device/status', timeout=3) as response:
        if json.load(response).get('recording'):
            raise SystemExit('仍有 Episode 在录制，请先结束录制再重启。')
    os.kill(pid, signal.SIGINT)
# uvicorn 会先关闭监听端口，再运行 shutdown（机械臂归零/释放 PCAN）。
# 必须等待原进程完全退出，不能只看端口是否空闲。
pid_list = [int(pid) for pid in pids]
try:
    wait_for_exit(pid_list, timeout=10)
except RuntimeError:
    # 浏览器页面还开着时，uvicorn 会无限等待 WebSocket 连接关闭；补发 SIGINT 强制退出。
    print('[restart] 旧服务仍在等待连接，补发 SIGINT…', flush=True)
    for pid in pid_list:
        os.kill(pid, signal.SIGINT)
    try:
        wait_for_exit(pid_list, timeout=10)
    except RuntimeError:
        # 兜底：上面已确认不在录制，SIGKILL 不会丢数据；机械臂保持最后姿态，PCAN 由系统回收。
        print('[restart] 仍未退出，改用 SIGKILL 兜底（机械臂保持当前姿态）', flush=True)
        for pid in pid_list:
            os.kill(pid, signal.SIGKILL)
        wait_for_exit(pid_list, timeout=10)
check = subprocess.run(['lsof', '-t', '-iTCP:8787', '-sTCP:LISTEN'], capture_output=True, text=True)
if check.stdout.strip():
    raise SystemExit('8787 已被其他服务占用，未启动第二个控制进程。')

PY
# 与随后启动的服务一起继承 Terminal 权限；只打开相机，不启用辅助运动。
.venv/bin/python - <<'PY' &
import json, time, urllib.request
base = 'http://127.0.0.1:8787'
for _ in range(120):
    try:
        with urllib.request.urlopen(base + '/api/health', timeout=1):
            break
    except OSError:
        time.sleep(.5)
else:
    raise SystemExit('服务未就绪，未恢复相机。')
for index, name in [(int(os.environ.get('REBOT_ARM_CAMERA', 0)), 'wrist'),
                        (1, 'scene')]:
    for path, body in [('/api/camera/open', {'index': index}),
                       ('/api/camera/alias', {'index': index, 'name': name})]:
        req = urllib.request.Request(base + path, data=json.dumps(body).encode(),
                                     headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(req, timeout=15) as response:
            print(path, response.read().decode(), flush=True)
PY
exec ./scripts/run_serve.command
