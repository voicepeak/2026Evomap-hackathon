"""手机姿态遥控器 —— 手机浏览器 → 本机 HTTP 服务 → 姿态 → 速度指令。

纯标准库（http.server + ssl），不依赖任何第三方包。

独立测试（不动机械臂，只看数值）：
    ~/.local/bin/uv run python phone_ctrl.py --print

集成到 hybrid_teleop.py：
    ... --phone            # 在同一进程里起服务
"""
from __future__ import annotations

import argparse
import json
import math
import os
import socket
import ssl
import subprocess
import sys
import threading
import tempfile
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# ── 配置 ──────────────────────────────────────────────────────────────────────


@dataclass
class PhoneConfig:
    port: int = 9210
    token: str = "rebot"
    dead_deg: float = 6.0        # 死区（度）
    full_deg: float = 35.0       # 到这个角度就是满速
    expo: float = 1.5            # 手感曲线（>1 更细腻）
    z_max: float = 0.08          # 高度速度上限 m/s
    yaw_max: float = 35.0        # J6 自转速度上限 °/s
    y_max: float = 0.06          # 左右速度上限 m/s（默认不用）
    stale_s: float = 0.5         # 超过这个时间没数据 → 全部归零
    https: bool = False          # 自签名 HTTPS（iOS 传感器可能要）


def _wrap180(d: float) -> float:
    d = math.fmod(d, 360.0)
    if d > 180.0:
        d -= 360.0
    elif d < -180.0:
        d += 360.0
    return d


def _curve_to_vel(deg: float, dead: float, full: float, vmax: float, expo: float) -> float:
    """角度 → 速度：死区内为 0，full 处到 vmax，中间按 expo 曲线。"""
    a = abs(deg)
    if a <= dead or vmax == 0.0:
        return 0.0
    if a >= full:
        f = 1.0
    else:
        f = (a - dead) / max(full - dead, 1e-6)
    return math.copysign((f ** expo) * vmax, deg)


# ── 状态 ──────────────────────────────────────────────────────────────────────


class PhoneState:
    """HTTP 线程写入，控制循环读取。所有访问都在锁内。"""

    def __init__(self, cfg: PhoneConfig) -> None:
        self.cfg = cfg
        self.lock = threading.Lock()
        self.t = 0.0                     # 最近收到数据的时间 (monotonic)
        self.seq = 0
        self.n_recv = 0
        self.client_ip = ""
        self.alpha = 0.0
        self.beta = 0.0
        self.gamma = 0.0
        self.beta0 = 0.0                 # 校准零点
        self.gamma0 = 0.0
        self.z_vel = 0.0
        self.yaw_vel = 0.0
        self.y_vel = 0.0
        self.invert_z = False
        self.invert_yaw = False
        self.use_yaw_axis = False        # 用手机水平转动控 Y（默认关）
        self.estop_req = 0               # 计数器：请求冻结
        self.resume_req = 0              # 计数器：请求解冻
        self.msgs: list[str] = []

    # -- 手机发来的控制命令 --
    def cmd(self, name: str, value=None) -> str:
        with self.lock:
            if name == "calib":
                self.beta0 = self.beta
                self.gamma0 = self.gamma
                self.msgs.append("已校准回中")
                return "已校准回中（当前姿态=零点）"
            if name == "invert_z":
                self.invert_z = bool(value)
                return f"高度方向 {'反向' if self.invert_z else '正向'}"
            if name == "invert_yaw":
                self.invert_yaw = bool(value)
                return f"自转方向 {'反向' if self.invert_yaw else '正向'}"
            if name == "axis_y":
                self.use_yaw_axis = bool(value)
                return f"左右轴使用手机水平转动: {'开' if self.use_yaw_axis else '关'}"
            if name == "estop":
                self.estop_req += 1
                self.z_vel = self.yaw_vel = self.y_vel = 0.0
                return "⛔ 已请求冻结"
            if name == "resume":
                self.resume_req += 1
                return "✅ 已请求解冻"
        return f"未知命令 {name}"

    # -- 手机姿态 --
    def update(self, alpha: float, beta: float, gamma: float, ip: str = "") -> None:
        cfg = self.cfg
        with self.lock:
            self.alpha, self.beta, self.gamma = alpha, beta, gamma
            db = _wrap180(beta - self.beta0)
            dg = _wrap180(gamma - self.gamma0)
            da = _wrap180(alpha)
            if self.invert_z:
                db = -db
            if self.invert_yaw:
                dg = -dg
            self.z_vel = _curve_to_vel(db, cfg.dead_deg, cfg.full_deg, cfg.z_max, cfg.expo)
            self.yaw_vel = _curve_to_vel(dg, cfg.dead_deg, cfg.full_deg, cfg.yaw_max, cfg.expo)
            if self.use_yaw_axis:
                self.y_vel = _curve_to_vel(da, cfg.dead_deg * 2, 60.0, cfg.y_max, cfg.expo)
            else:
                self.y_vel = 0.0
            self.t = time.monotonic()
            self.seq += 1
            self.n_recv += 1
            if ip:
                self.client_ip = ip

    def snapshot(self) -> dict:
        with self.lock:
            age = (time.monotonic() - self.t) if self.t else float("inf")
            return {
                "age": round(age, 3),
                "fresh": age < self.cfg.stale_s,
                "seq": self.seq,
                "n": self.n_recv,
                "ip": self.client_ip,
                "alpha": round(self.alpha, 2),
                "beta": round(self.beta, 2),
                "gamma": round(self.gamma, 2),
                "dbeta": round(_wrap180(self.beta - self.beta0), 2),
                "dgamma": round(_wrap180(self.gamma - self.gamma0), 2),
                "z_vel": round(self.z_vel, 4),
                "yaw_vel": round(self.yaw_vel, 2),
                "y_vel": round(self.y_vel, 4),
                "invert_z": self.invert_z,
                "invert_yaw": self.invert_yaw,
                "axis_y": self.use_yaw_axis,
                "estop_req": self.estop_req,
                "resume_req": self.resume_req,
            }


# ── 网页 ──────────────────────────────────────────────────────────────────────

PAGE_HTML = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>reBot 手机姿态遥控</title>
<style>
  :root { color-scheme: dark; }
  * { box-sizing: border-box; -webkit-user-select: none; user-select: none; }
  body { margin:0; padding:14px; font:16px/1.45 -apple-system,system-ui,"PingFang SC",sans-serif;
         background:#0b1220; color:#e5e7eb; }
  h1 { font-size:19px; margin:0 0 10px; }
  .card { background:#111a2e; border:1px solid #1f2a44; border-radius:14px; padding:12px; margin-bottom:10px; }
  .row { display:flex; justify-content:space-between; gap:8px; padding:3px 0; }
  .k { color:#93a4c4; }
  .v { font-variant-numeric:tabular-nums; font-weight:700; }
  .big { font-size:26px; }
  .ok { color:#34d399; } .warn { color:#fbbf24; } .bad { color:#f87171; }
  button { font:inherit; font-weight:700; padding:13px 12px; border-radius:12px;
           border:1px solid #2b3a5c; background:#1b2742; color:#e5e7eb; flex:1; }
  button:active { transform:scale(.98); }
  .grid { display:grid; grid-template-columns:1fr 1fr; gap:8px; }
  .estop { background:#7f1d1d; border-color:#ef4444; }
  .go { background:#14532d; border-color:#22c55e; }
  #pad { height:190px; border:2px dashed #2b3a5c; border-radius:14px; display:flex;
         align-items:center; justify-content:center; color:#64748b; text-align:center; }
  #log { font:12px/1.5 ui-monospace,monospace; color:#8aa0c6; white-space:pre-wrap; min-height:32px;
         max-height:120px; overflow:auto; }
  .hint { font-size:12.5px; color:#8aa0c6; }
</style>
</head>
<body>
<h1>🤖 reBot 手机姿态遥控</h1>

<div class="card">
  <div class="row"><span class="k">传感器</span><span class="v" id="s-sensor">未启用</span></div>
  <div class="row"><span class="k">发送频率</span><span class="v" id="s-fps">0 Hz</span></div>
  <div class="row"><span class="k">前后倾 → 高度</span><span class="v big" id="s-z">0.000 m/s</span></div>
  <div class="row"><span class="k">左右倾 → 自转</span><span class="v big" id="s-yaw">0.0 °/s</span></div>
  <div class="row"><span class="k">原始 b / g</span><span class="v" id="s-raw">—</span></div>
</div>

<div class="grid" style="margin-bottom:10px">
  <button class="go" id="b-start">① 启用传感器</button>
  <button id="b-calib">② 校准回中</button>
</div>
<div class="grid" style="margin-bottom:10px">
  <button id="b-iz">高度反向</button>
  <button id="b-iy">自转反向</button>
</div>
<div class="grid" style="margin-bottom:10px">
  <button class="estop" id="b-stop">⛔ 冻结机械臂</button>
  <button id="b-resume">✅ 解除冻结</button>
</div>

<div class="card">
  <div class="hint" style="margin-bottom:6px">没有传感器权限时，用下面的触摸板（水平=自转，垂直=高度）</div>
  <div id="pad">触摸拖动 = 摇杆</div>
</div>

<div class="card"><div id="log">等待操作…</div></div>

<script>
const TOKEN = new URLSearchParams(location.search).get('t') || '';
const $ = id => document.getElementById(id);
let seq = 0, sending = false, last = null, okN = 0, fpsT = performance.now(), hz = 0;
let sensorOn = false, padX = 0, padY = 0, sent = 0;

function log(s) { $('log').textContent = s; }
function set(id, txt, cls) { const e = $(id); e.textContent = txt; e.className = 'v' + (cls ? ' ' + cls : ''); }

function post(body) {
  body.t = TOKEN;
  return fetch('/pose', { method:'POST', headers:{'Content-Type':'application/json'},
                          body: JSON.stringify(body), cache:'no-store' })
    .then(r => r.json()).catch(() => null);
}
function cmd(name, value) {
  post({ cmd: name, value: value }).then(r => { if (r) log(r.msg || 'ok'); });
}

// ── 30Hz 发送 ──
setInterval(() => {
  if (sending) return;
  let a, b, g;
  if (sensorOn && last) { a = last.a; b = last.b; g = last.g; }
  else if (padY || padX) { a = 0; b = padY * 40; g = padX * 40; }   // 触摸摇杆兜底
  else return;
  sending = true;
  sent = Date.now();
  post({ a:a, b:b, g:g, s:++seq }).then(r => {
    sending = false;
    okN++;
    const now = performance.now();
    if (now - fpsT > 500) { hz = Math.round(okN * 1000 / (now - fpsT)); okN = 0; fpsT = now; set('s-fps', hz + ' Hz'); }
    if (!r) { set('s-sensor', '⚠️ 服务器无响应', 'bad'); return; }
    set('s-z', (r.z_vel >= 0 ? '+' : '') + r.z_vel.toFixed(3) + ' m/s', r.z_vel ? 'ok' : '');
    set('s-yaw', (r.yaw_vel >= 0 ? '+' : '') + r.yaw_vel.toFixed(1) + ' °/s', r.yaw_vel ? 'ok' : '');
    set('s-raw', (b.toFixed(1)) + '° / ' + (g.toFixed(1)) + '°');
  });
}, 33);

// ── 传感器 ──
function onOri(e) {
  if (e.beta === null || e.gamma === null) { set('s-sensor', '❌ 收不到数据', 'bad'); return; }
  last = { a: e.alpha || 0, b: e.beta, g: e.gamma };
  sensorOn = true;
  set('s-sensor', '✅ 已连接', 'ok');
}
function startSensor() {
  if (typeof DeviceOrientationEvent === 'undefined') {
    set('s-sensor', '❌ 浏览器不支持', 'bad'); sensorOn = false; return;
  }
  if (typeof DeviceOrientationEvent.requestPermission === 'function') {
    DeviceOrientationEvent.requestPermission().then(r => {
      if (r === 'granted') { window.addEventListener('deviceorientation', onOri); set('s-sensor', '已授权，等数据…', 'warn'); }
      else { set('s-sensor', '❌ 权限被拒绝', 'bad'); log('权限被拒绝：请用 HTTPS 打开本页，或改用触摸板'); }
    }).catch(err => { set('s-sensor', '❌ ' + err, 'bad'); log('iOS 需要 HTTPS：' + err); });
  } else {
    window.addEventListener('deviceorientation', onOri);
    set('s-sensor', '已启动，等数据…', 'warn');
  }
}

// ── 触摸摇杆（兜底）──
const pad = $('pad');
let padId = null, cx = 0, cy = 0;
function padStart(x, y) { const r = pad.getBoundingClientRect(); cx = r.left + r.width/2; cy = r.top + r.height/2; padMove(x, y); }
function padMove(x, y) {
  const r = pad.getBoundingClientRect();
  padX = Math.max(-1, Math.min(1, (x - cx) / (r.width/2)));
  padY = Math.max(-1, Math.min(1, (cy - y) / (r.height/2)));
  pad.textContent = '高度 ' + (padY*40).toFixed(0) + '°  自转 ' + (padX*40).toFixed(0) + '°';
}
pad.addEventListener('touchstart', e => { e.preventDefault(); const t = e.changedTouches[0]; padId = t.identifier; padStart(t.clientX, t.clientY); }, {passive:false});
pad.addEventListener('touchmove', e => { e.preventDefault(); for (const t of e.changedTouches) if (t.identifier === padId) padMove(t.clientX, t.clientY); }, {passive:false});
pad.addEventListener('touchend', e => { padId = null; padX = padY = 0; pad.textContent = '触摸拖动 = 摇杆'; }, {passive:false});
pad.addEventListener('mousedown', e => { padStart(e.clientX, e.clientY);
  const mm = ev => padMove(ev.clientX, ev.clientY);
  const mu = () => { document.removeEventListener('mousemove', mm); document.removeEventListener('mouseup', mu); padX = padY = 0; pad.textContent = '触摸拖动 = 摇杆'; };
  document.addEventListener('mousemove', mm); document.addEventListener('mouseup', mu); });

// ── 按钮 ──
$('b-start').onclick = () => { startSensor(); log('已请求传感器权限'); };
$('b-calib').onclick = () => cmd('calib');
$('b-iz').onclick = () => cmd('invert_z', true);      // 简化：按一次置为反向
$('b-iy').onclick = () => cmd('invert_yaw', true);
$('b-stop').onclick = () => cmd('estop');
$('b-resume').onclick = () => cmd('resume');
log('页面已加载。① 先点"启用传感器"，② 再点"校准回中"（把当前姿势设为中位）');
</script>
</body>
</html>
"""


# ── HTTP 服务 ─────────────────────────────────────────────────────────────────


def make_handler(state: PhoneState, cfg: PhoneConfig):
    class Handler(BaseHTTPRequestHandler):
        server_version = "rebot-phone/1.0"
        protocol_version = "HTTP/1.1"

        def _send(self, code: int, ctype: str, body: bytes) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(body)
            except Exception:  # noqa: BLE001
                pass

        def do_GET(self) -> None:  # noqa: N802
            path = self.path.split("?")[0]
            if path in ("/", "/index.html"):
                self._send(200, "text/html; charset=utf-8", PAGE_HTML.encode("utf-8"))
            elif path == "/status":
                self._send(200, "application/json", json.dumps(state.snapshot()).encode())
            elif path == "/favicon.ico":
                self._send(204, "image/x-icon", b"")
            else:
                self._send(404, "text/plain; charset=utf-8", "not found".encode())

        def do_POST(self) -> None:  # noqa: N802
            if self.path.split("?")[0] != "/pose":
                self._send(404, "text/plain; charset=utf-8", "not found".encode())
                return
            try:
                n = int(self.headers.get("Content-Length") or 0)
            except ValueError:
                n = 0
            raw = self.rfile.read(n) if n > 0 else b"{}"
            try:
                d = json.loads(raw.decode("utf-8", "ignore") or "{}")
            except Exception:  # noqa: BLE001
                self._send(400, "application/json", b'{"ok":0,"msg":"bad json"}')
                return
            if cfg.token and str(d.get("t", "")) != cfg.token:
                self._send(403, "application/json", b'{"ok":0,"msg":"token"}')
                return
            ip = self.client_address[0]
            msg = ""
            if d.get("cmd"):
                msg = state.cmd(str(d["cmd"]), d.get("value"))
            else:
                try:
                    state.update(float(d.get("a", 0.0)), float(d.get("b", 0.0)),
                                 float(d.get("g", 0.0)), ip)
                except (TypeError, ValueError):
                    pass
            snap = state.snapshot()
            out = {"ok": 1, "msg": msg, "z_vel": snap["z_vel"], "yaw_vel": snap["yaw_vel"],
                   "y_vel": snap["y_vel"], "age": snap["age"]}
            self._send(200, "application/json", json.dumps(out).encode())

        def log_message(self, *a) -> None:  # 静音，避免刷 Terminal
            pass

    return Handler


def _lan_ip() -> str:
    """猜本机在局域网里的地址（让手机能访问）。"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:  # noqa: BLE001
        try:
            return socket.gethostbyname(socket.gethostname())
        except Exception:  # noqa: BLE001
            return "127.0.0.1"


def _ensure_cert(ip: str) -> tuple[str, str]:
    """生成自签名证书（iOS 的传感器在 HTTPS 下才放行）。"""
    d = os.path.join(tempfile.gettempdir(), "rebot_phone_cert")
    os.makedirs(d, exist_ok=True)
    key, crt = os.path.join(d, "key.pem"), os.path.join(d, "cert.pem")
    if os.path.exists(key) and os.path.exists(crt):
        return crt, key
    cnf = os.path.join(d, "openssl.cnf")
    with open(cnf, "w") as f:
        f.write(f"""[req]
distinguished_name=dn
req_extensions=v3_req
prompt=no
[dn]
CN={ip}
[v3_req]
subjectAltName=IP:{ip},IP:127.0.0.1,DNS:localhost
basicConstraints=CA:TRUE
""")
    subprocess.run(["openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes",
                    "-days", "3650", "-keyout", key, "-out", crt, "-config", cnf],
                   check=True, capture_output=True)
    return crt, key


class PhoneServer:
    def __init__(self, cfg: PhoneConfig, state: PhoneState | None = None) -> None:
        self.cfg = cfg
        self.state = state or PhoneState(cfg)
        self.httpd: ThreadingHTTPServer | None = None
        self.url = ""

    def start(self) -> str:
        self.httpd = ThreadingHTTPServer(("0.0.0.0", self.cfg.port), make_handler(self.state, self.cfg))
        self.httpd.daemon_threads = True
        scheme = "http"
        if self.cfg.https:
            crt, key = _ensure_cert(_lan_ip())
            ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            ctx.load_cert_chain(crt, key)
            self.httpd.socket = ctx.wrap_socket(self.httpd.socket, server_side=True)
            scheme = "https"
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        tok = f"?t={self.cfg.token}" if self.cfg.token else ""
        self.url = f"{scheme}://{_lan_ip()}:{self.cfg.port}/{tok}"
        return self.url

    def stop(self) -> None:
        if self.httpd is not None:
            try:
                self.httpd.shutdown()
            except Exception:  # noqa: BLE001
                pass


# ── 独立测试 ──────────────────────────────────────────────────────────────────


def main() -> None:
    ap = argparse.ArgumentParser(description="手机姿态遥控（独立测试模式）")
    ap.add_argument("--port", type=int, default=9210)
    ap.add_argument("--token", default="rebot")
    ap.add_argument("--https", action="store_true")
    ap.add_argument("--z-max", type=float, default=0.08)
    ap.add_argument("--yaw-max", type=float, default=35.0)
    ap.add_argument("--dead", type=float, default=6.0)
    ap.add_argument("--print", action="store_true", help="每秒打印一次收到的姿态与速度")
    args = ap.parse_args()

    cfg = PhoneConfig(port=args.port, token=args.token, https=args.https,
                      z_max=args.z_max, yaw_max=args.yaw_max, dead_deg=args.dead)
    srv = PhoneServer(cfg)
    url = srv.start()
    print(f"✅ 手机遥控页面: {url}")
    print("   手机上打开这个地址 → ① 启用传感器 ② 校准回中")
    print("   （Mac 端按 Ctrl+C 结束）", flush=True)
    try:
        while True:
            time.sleep(1.0)
            if args.print:
                s = srv.state.snapshot()
                if s["n"] == 0:
                    print(f"[phone] 还没收到数据  (age={s['age']})", flush=True)
                else:
                    print(f"[phone] n={s['n']:5d} age={s['age']:.2f}s  "
                          f"b={s['beta']:+7.1f}° g={s['gamma']:+7.1f}°  "
                          f"Δb={s['dbeta']:+7.1f}° Δg={s['dgamma']:+7.1f}°  → "
                          f"z={s['z_vel']:+.3f} m/s  yaw={s['yaw_vel']:+6.1f} °/s", flush=True)
    except KeyboardInterrupt:
        print("\n[phone] 结束")
    finally:
        srv.stop()


if __name__ == "__main__":
    sys.exit(main())
