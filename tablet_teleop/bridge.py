#!/usr/bin/env python3
"""tablet_teleop · bridge —— 数位板 → Python 的桥

M0 阶段只做三件事：
  1. 收网页发来的笔样本（WebSocket）
  2. 维护一个线程安全的环形缓冲，供控制循环读取（M1 的接口）
  3. 实时打印采样率统计 / 可选写 CSV

M1 之后：在 ArmSink 里接上 reBotArm_control_py。

用法：
    pip install websockets
    python bridge.py                 # 只打印
    python bridge.py --csv out.csv   # 顺便记录

然后在浏览器打开 web/index.html，点「连接 bridge」。
"""
from __future__ import annotations

import argparse
import asyncio
import csv
import json
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque, Optional

try:                                    # websockets >= 13
    from websockets.asyncio.server import serve
except ImportError:                     # 老版本兜底
    from websockets.server import serve

HOST = "127.0.0.1"
PORT = 8765

CSV_HEADER = ["t_page_ms", "x_mm", "y_mm", "pressure", "tiltX_deg", "tiltY_deg",
              "altitude_rad", "azimuth_rad", "twist_deg", "width", "height",
              "touching", "pointer_type", "F_des_N"]


# ──────────────────────────────────────────────────────────────────────
# 样本
# ──────────────────────────────────────────────────────────────────────

@dataclass
class PenSample:
    """一帧笔样本。字段名与 web/index.html 里的一致。"""
    t: float          # 浏览器 performance.now()，ms（页面内单调）
    ts: float         # 原始 PointerEvent.timeStamp，ms
    x: float          # 板面 mm
    y: float          # 板面 mm
    p: float          # 笔压 0~1
    tx: float         # tiltX 度
    ty: float         # tiltY 度
    alt: Optional[float]
    az: Optional[float]
    tw: float
    w: float
    h: float
    type: str
    touching: bool
    F: float          # 映射出的法向力指令 N


class RingBuffer:
    """线程安全的最近样本缓冲。

    控制循环（另一个线程）调 latest() 拿最新样本，
    调 age_ms() 做看门狗。

    注意时钟：s.t 是浏览器的 performance.now()，与 Python 的
    perf_counter() 原点不同，**不能直接相减**。看门狗一律用
    age_ms()（基于样本到达 Python 的本地时刻）。
    """

    def __init__(self, maxlen: int = 8192) -> None:
        self._buf: Deque[PenSample] = deque(maxlen=maxlen)
        self._lock = threading.Lock()
        self._t_arrive: float = 0.0      # 本地单调时钟
        self.total: int = 0

    def push(self, s: PenSample) -> None:
        with self._lock:
            self._buf.append(s)
            self._t_arrive = time.perf_counter()
            self.total += 1

    def latest(self) -> Optional[PenSample]:
        with self._lock:
            return self._buf[-1] if self._buf else None

    def age_ms(self) -> float:
        """距最近一次样本到达过了多久（ms）。看门狗用。"""
        with self._lock:
            t = self._t_arrive
        if not t:
            return float("inf")
        return (time.perf_counter() - t) * 1000.0

    def last_n(self, n: int) -> list[PenSample]:
        with self._lock:
            return list(self._buf)[-n:]


BUF = RingBuffer()


# ──────────────────────────────────────────────────────────────────────
# 消费端：M1 在这里接机械臂
# ──────────────────────────────────────────────────────────────────────

class ArmSink:
    """M1 的插槽。现在只计数。

    M1 时把 reBotArm 接进来：

        from reBotArm_control_py.actuator import RebotArm
        arm = RebotArm(); arm.connect()
        arm.arm.mode_mit(); arm.enable_all()
        arm.start_control_loop(self._loop)

    控制循环里：读 BUF.latest()，算 q/dq/tau_g/F_ext，导纳积分出 z，
    再 arm.arm.send_mit(pos, vel, kp, kd, tau)。
    """

    def __init__(self) -> None:
        self.n = 0

    def on_sample(self, s: PenSample) -> None:
        self.n += 1
        # TODO(M1)


SINK = ArmSink()


# ──────────────────────────────────────────────────────────────────────
# CSV
# ──────────────────────────────────────────────────────────────────────

class CsvSink:
    def __init__(self, path: Path) -> None:
        self.path = path
        self.fh = path.open("w", newline="", encoding="utf-8")
        self.w = csv.writer(self.fh)
        self.w.writerow(CSV_HEADER)
        self.n = 0
        self._lock = threading.Lock()

    def write(self, s: PenSample) -> None:
        with self._lock:
            self.w.writerow([
                f"{s.t:.3f}", f"{s.x:.4f}", f"{s.y:.4f}", f"{s.p:.5f}",
                f"{s.tx:.2f}", f"{s.ty:.2f}",
                "" if s.alt is None else f"{s.alt:.5f}",
                "" if s.az is None else f"{s.az:.5f}",
                f"{s.tw:.1f}", f"{s.w:.2f}", f"{s.h:.2f}",
                1 if s.touching else 0, s.type, f"{s.F:.4f}",
            ])
            self.n += 1

    def flush(self) -> None:
        with self._lock:
            self.fh.flush()

    def close(self) -> None:
        with self._lock:
            self.fh.flush()
            self.fh.close()


CSV: Optional[CsvSink] = None


# ──────────────────────────────────────────────────────────────────────
# WebSocket
# ──────────────────────────────────────────────────────────────────────

def parse(raw: str) -> Optional[PenSample]:
    try:
        d = json.loads(raw)
    except json.JSONDecodeError:
        return None
    try:
        return PenSample(
            t=float(d.get("t", 0.0)),
            ts=float(d.get("ts", 0.0)),
            x=float(d.get("x", 0.0)),
            y=float(d.get("y", 0.0)),
            p=float(d.get("p", 0.0)),
            tx=float(d.get("tx", 0.0)),
            ty=float(d.get("ty", 0.0)),
            alt=None if d.get("alt") is None else float(d["alt"]),
            az=None if d.get("az") is None else float(d["az"]),
            tw=float(d.get("tw", 0.0)),
            w=float(d.get("w", 0.0)),
            h=float(d.get("h", 0.0)),
            type=str(d.get("type", "?")),
            touching=bool(d.get("touching", False)),
            F=float(d.get("F", 0.0)),
        )
    except (TypeError, ValueError):
        return None


async def handler(ws) -> None:
    peer = getattr(ws, "remote_address", None)
    where = f"{peer[0]}:{peer[1]}" if peer else "?"
    print(f"[bridge] 网页已连接 {where}")
    try:
        async for raw in ws:
            s = parse(raw)
            if s is None:
                continue
            BUF.push(s)
            SINK.on_sample(s)
            if CSV is not None:
                CSV.write(s)
    except Exception as e:                      # 断开是常态
        print(f"[bridge] 断开：{type(e).__name__}: {e}")


async def main_async(csv_path: Optional[Path]) -> None:
    global CSV
    if csv_path:
        CSV = CsvSink(csv_path)
        print(f"[bridge] 写入 {csv_path}")

    async with serve(handler, HOST, PORT, max_size=2 ** 20):
        print(f"[bridge] 监听 ws://{HOST}:{PORT}")
        print("[bridge] 打开 web/index.html，点「连接 bridge」。Ctrl+C 退出。")
        print("-" * 66)
        print(f"{'rate(Hz)':>9} {'total':>8} {'age(ms)':>8} {'type':>6} "
              f"{'p':>7} {'F(N)':>7} {'touch':>6}")
        print("-" * 66)

        t_prev, n_prev = time.perf_counter(), 0
        try:
            while True:
                await asyncio.sleep(1.0)
                now = time.perf_counter()
                n = BUF.total
                rate = (n - n_prev) / max(now - t_prev, 1e-6)
                t_prev, n_prev = now, n

                if CSV is not None:
                    CSV.flush()

                s = BUF.latest()
                age = BUF.age_ms()
                if s is None:
                    print(f"{rate:9.1f} {n:8d} {age:8.0f} {'—':>6}")
                    continue
                print(f"{rate:9.1f} {n:8d} {age:8.1f} {s.type:>6} "
                      f"{s.p:7.3f} {s.F:7.2f} {'●' if s.touching else '○':>6}")
        finally:
            if CSV is not None:
                CSV.close()
                print(f"[bridge] CSV 已写入 {CSV.n} 行 → {CSV.path}")


def main() -> None:
    ap = argparse.ArgumentParser(description="数位板 → Python 桥")
    ap.add_argument("--csv", type=Path, default=None, help="把样本写成 CSV")
    args = ap.parse_args()
    try:
        asyncio.run(main_async(args.csv))
    except KeyboardInterrupt:
        print("\n[bridge] 退出")


if __name__ == "__main__":
    main()
