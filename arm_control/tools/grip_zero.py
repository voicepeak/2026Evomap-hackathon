#!/usr/bin/env python3
"""grip_zero.py — 夹爪零点重设 + 行程复测（**只动夹爪，不动机械臂**）。

背景：夹爪电机的"零点"是标定出来的，约定是 **0° = 爪片完全合上**
（见《整体方案》的标定流程："手动摆到零位、夹爪闭合、回车"）。这台夹爪自锁、
不通电掰不动，所以"摆到合上位置"只能让电机自己慢慢顶到机械端，再把那里记成 0°。

现场遇到过的问题：某台机械上 **角度增大 = 夹紧**（和软件的"角度增大 = 张开"相反），
所以"程序里的 3° 闭合端"其实是**张开**端。做法：
    ① 用本工具把"合到位"设成 0°；
    ② 在配置里把夹爪方向反过来（`gripper.direction = -1`）+ 填新的 lo_deg/hi_deg；
       `--write-config` 可以直接写进 capture_platform/configs/rebot_b601_rs.json。

流程（按 --dir 给的方向，先自己确认那是"合"）：
    连接（只清故障码，不使能机械臂）→ 只给夹爪这一路电机使能 →
    按 --speed 慢速推进，目标最多比实测超前 --lead-deg（kp×偏差 = 力矩上限，
    永远不会顶着堵转）→ 力矩超限 / 推着不动 / 到行程上限 → 停
    → --set-zero：把当前位置记为 0° 并保存参数（断电不丢）
    → --span：再往反方向走到另一端，量整程并给出配置建议

用法（**先停掉采集服务**；PCAN 被服务独占时连不上）：
    cd <工作区根>
    # 1) 先确认哪个方向是"合"（慢速，随时 Ctrl+C）：
    capture_platform/.venv/bin/python arm_control/tools/grip_zero.py --dir +1 --speed 0.4
    #    爪片在合 → 方向对了（这台就是 +1）；在开 → 换 --dir -1
    # 2) 一步到位：设零点 + 量整程 + 写配置
    capture_platform/.venv/bin/python arm_control/tools/grip_zero.py --dir +1 --set-zero --span --write-config
    # 3) 重启服务，测：握拳=合、张开手=开、笔左划=合、右划=开

安全：只 enable/命令夹爪这一路电机；机械臂 6 个关节全程不使能、不下发。
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

# 本仓库根（arm_control/）；可用环境变量覆盖
REPO = os.environ.get("REBOT_ARM_REPO") or str(Path(__file__).resolve().parents[1])
sys.path.insert(0, REPO)

from motorbridge import Mode  # noqa: E402

from reBotArm_control_py.actuator import RebotArm  # noqa: E402

RATE = 100.0


def write_config(path: str, direction: int, lo_deg: float, hi_deg: float | None) -> tuple[bool, str]:
    """把标定结果写进机型配置的 gripper 段（先备份 .bak）。返回 (是否成功, 说明)。"""
    import json
    import shutil

    p = Path(path)
    if not p.exists():
        return False, f"找不到配置：{p}"
    try:
        raw = json.loads(p.read_text(encoding="utf-8"))
    except Exception as e:  # noqa: BLE001
        return False, f"配置读不出：{e}"
    g = raw.get("gripper")
    if not isinstance(g, dict):
        return False, f"配置里没有 gripper 段：{p}"
    shutil.copy2(p, p.with_suffix(p.suffix + ".bak"))
    g["direction"] = int(direction)
    if hi_deg is not None:
        g["lo_deg"] = round(float(lo_deg), 1)
        g["hi_deg"] = round(float(hi_deg), 1)
    p.write_text(json.dumps(raw, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    keys = f'direction={g["direction"]}' + (f' lo_deg={g.get("lo_deg")} hi_deg={g.get("hi_deg")}'
                                            if hi_deg is not None else "")
    return True, f"{p}（{keys}；备份 {p.name}.bak）"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", type=int, default=-1, help="-1 / +1：往哪个方向走（这台通常 −1 = 角度减小 = 合）")
    ap.add_argument("--speed", type=float, default=0.4, help="推进速度 rad/s（慢一点更安全）")
    ap.add_argument("--max-travel", type=float, default=7.0, help="最多走多少 rad（约两圈，兜底）")
    ap.add_argument("--tau-limit", type=float, default=1.2, help="力矩上限 N·m：到它就认为到机械端")
    ap.add_argument("--lead-deg", type=float, default=4.0, help="目标最多比实测超前多少度（=力矩上限/kp）")
    ap.add_argument("--kp", type=float, default=20.0)
    ap.add_argument("--kd", type=float, default=2.0)
    ap.add_argument("--set-zero", action="store_true", help="到端后把当前位置设为 0° 并保存参数")
    ap.add_argument("--span", action="store_true", help="设完零点再往反方向走到另一端，量整程")
    ap.add_argument("--lo-margin", type=float, default=4.0, help="闭合端余量（度）")
    ap.add_argument("--hi-margin", type=float, default=30.0,
                    help="开口端余量（度）：留大一些，避免经常卡在外面")
    ap.add_argument("--write-config", nargs="?", const="auto", default=None,
                    help="把建议的 direction/lo_deg/hi_deg 写进机型配置（默认 "
                         "<工作区>/capture_platform/configs/rebot_b601_rs.json，先备份 .bak）")
    ap.add_argument("--hold-s", type=float, default=0.4, help="结束时保持一下再失能（秒）")
    args = ap.parse_args()

    if args.dir not in (-1, 1):
        print("--dir 只能是 -1 或 +1")
        return 2

    arm = RebotArm()
    arm.connect()                      # 只做 clear_error / 建总线，不使能
    group = arm.gripper
    name = group.joint_names[0]
    mot = arm._motor_map[name]

    def poll() -> None:
        """刷新反馈（和平台 real_arm 里一样：请求 + 让控制器收一帧）。"""
        for m in arm._motor_map.values():
            try:
                m.request_feedback()
            except Exception:  # noqa: BLE001
                pass
        for c in arm._ctrl_map.values():
            try:
                c.poll_feedback_once()
            except Exception:  # noqa: BLE001
                pass

    def read() -> tuple[float, float]:
        poll()
        time.sleep(0.01)
        st = mot.get_state()
        if st is None:
            return float("nan"), 0.0
        return float(st.pos), float(st.torq)

    def send(pos_target: float) -> None:
        mot.send_mit(float(pos_target), 0.0, float(args.kp), float(args.kd), 0.0)

    def travel(direction: int, tag: str) -> tuple[float, float, bool]:
        """单向推进。返回 (起点角, 停止角, 是否到端)。"""
        start, _ = read()
        target = start
        dt = 1.0 / RATE
        lead = math.radians(args.lead_deg)
        last_log = 0.0
        last_pos = start
        stall_since: float | None = None
        t0 = time.perf_counter()
        hit_end = False
        while True:
            target += direction * args.speed * dt
            pos, tau = read()
            send(float(np.clip(target, pos - lead, pos + lead)))     # 限力：偏差 ≤ lead
            time.sleep(dt)
            now = time.perf_counter()
            if abs(pos - last_pos) > math.radians(0.3):
                last_pos = pos
                stall_since = None
            elif stall_since is None:
                stall_since = now
            if now - last_log > 0.3:
                last_log = now
                print(f"  [{tag}] 实测 {math.degrees(pos):+8.1f}°  目标 {math.degrees(target):+8.1f}°  "
                      f"力矩 {tau:+5.2f}N·m", flush=True)
            if abs(tau) > args.tau_limit:
                print(f"  → 力矩 {tau:+.2f}N·m 超阈值 {args.tau_limit}N·m：到机械端了", flush=True)
                hit_end = True
                break
            if stall_since is not None and now - stall_since > 0.6:
                print("  → 推着不动 0.6s：当作到端了", flush=True)
                hit_end = True
                break
            if abs(pos - start) > args.max_travel:
                print(f"  → 已走 {math.degrees(abs(pos - start)):.0f}° 超过上限：停（可能方向反了/没到端）",
                      flush=True)
                break
            if now - t0 > 60:
                print("  → 60s 超时：停", flush=True)
                break
        pos_end, _ = read()
        send(pos_end)                  # 停手：目标=实测（自锁机构不耗力也能保持）
        time.sleep(0.2)
        return start, pos_end, hit_end

    pos0, tau0 = read()
    print(f"夹爪电机 name={name}（motor_id=7）  当前 {math.degrees(pos0):+.1f}°  力矩 {tau0:+.2f}N·m")
    print(f"方向 {args.dir:+d}  速度 {args.speed} rad/s  力矩上限 {args.tau_limit}N·m  "
          f"目标超前 ≤ {args.lead_deg}°（≈{args.kp * math.radians(args.lead_deg):.1f}N·m 内）")
    print("（机械臂关节不使能、不下发；随时 Ctrl+C 停）\n", flush=True)

    mot.ensure_mode(Mode.MIT, 1000)
    mot.enable()
    time.sleep(0.2)
    try:
        start, end, hit = travel(args.dir, "推进")
        print(f"\n⏹ {math.degrees(start):+.1f}° → {math.degrees(end):+.1f}°"
              f"（走了 {math.degrees(end - start):+.1f}°，{'到端' if hit else '未确认到端'}）", flush=True)

        if args.set_zero and hit:
            mot.set_zero_position()
            time.sleep(0.1)
            try:
                mot.store_parameters()          # 保存到电机，断电不丢
                saved = "，并已保存参数（断电不丢）"
            except Exception as e:  # noqa: BLE001
                saved = f"（⚠️ 保存参数失败：{e}；零点可能断电丢失）"
            pos_after, _ = read()
            print(f"✅ 已把当前位置设为 0°{saved}；回读 = {math.degrees(pos_after):+.2f}°", flush=True)
            print("   约定：0° = 爪片合到位；程序里的 lo_deg/hi_deg 都相对这个 0°。", flush=True)

        if args.span and hit:
            s2, e2, hit2 = travel(-args.dir, "反向")
            span = abs(e2 - s2)
            print(f"\n📏 反向到另一端：{math.degrees(s2):+.1f}° → {math.degrees(e2):+.1f}°"
                  f"（{'到端' if hit2 else '未确认到端'}）  整程 ≈ {math.degrees(span):.0f}°", flush=True)

        if args.set_zero and hit:
            # 0° 已 = 合到位。软件约定"角度增大 = 张开"：
            # 若"合"的方向是 +1（角度增大），就要把配置里的 direction 反过来。
            direction = -1 if args.dir > 0 else 1
            span_deg = math.degrees(abs(e2 - s2)) if (args.span and hit2) else None
            lo = float(args.lo_margin)
            hi = max(lo + 10.0, (span_deg - float(args.hi_margin))) if span_deg else None
            print("   → capture_platform/configs/rebot_b601_rs.json 的 gripper 建议：", flush=True)
            print(f'     "direction": {direction},', flush=True)
            if hi is not None:
                print(f'     "lo_deg": {lo:.0f},  "hi_deg": {hi:.0f}   '
                      f'（0° = 合到位；闭合端留 {lo:.0f}°、开口端留 {args.hi_margin:.0f}° 余量）',
                      flush=True)
            else:
                print('     "lo_deg" / "hi_deg"：再加 --span 量整程后给', flush=True)
            if args.write_config is not None:
                path = args.write_config
                if path == "auto":
                    path = str(Path(REPO).resolve().parent / "capture_platform" / "configs"
                               / "rebot_b601_rs.json")
                ok, msg = write_config(path, direction, lo, hi)
                print(("   ✅ 已写入 " if ok else "   ⚠️ 未写入 ") + msg, flush=True)
            else:
                print("   ⚠️ 硬件零点已改，配置还没改（--write-config 可自动写）。"
                      "不写就重启服务的话，程序会往「夹紧」方向顶，请务必改完再重启。", flush=True)
    finally:
        t_hold = time.perf_counter()
        while time.perf_counter() - t_hold < max(0.0, args.hold_s):
            p, _ = read()
            send(p)
            time.sleep(0.02)
        mot.disable()
        print("\n✅ 夹爪已失能（机械臂关节全程未使能、未下发）", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
