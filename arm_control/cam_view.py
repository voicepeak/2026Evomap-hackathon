#!/usr/bin/env python3
"""Open every available camera and show live previews in one window.

Probes camera indices 0..N (AVFoundation), reads frames in a thread per camera,
tiles the previews in a Tk window.  Keys: S = save snapshots, Q/Esc = quit.
"""
from __future__ import annotations

import sys
import threading
import time
import tkinter as tk
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageTk

MAX_PROBE = 5
SNAP_DIR = Path("/Users/Admin/Desktop/reBotArm_control_py/logs/cam_snapshots")


class Cam:
    def __init__(self, idx: int) -> None:
        self.idx = idx
        self.cap: cv2.VideoCapture | None = None
        self.frame: np.ndarray | None = None
        self.lock = threading.Lock()
        self.ok = False
        self.info = "未打开"
        self.fps = 0.0
        self.n = 0
        self.running = False
        self.thread: threading.Thread | None = None

    def open(self) -> bool:
        cap = None
        for backend in (cv2.CAP_AVFOUNDATION, cv2.CAP_ANY):
            cap = cv2.VideoCapture(self.idx, backend)
            if cap.isOpened():
                break
            cap.release()
            cap = None
        if cap is None:
            self.info = "打不开"
            return False
        for w, h in ((1280, 720), (640, 480), (0, 0)):
            if w:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            t0 = time.time()
            ok, frame = cap.read()
            if ok and frame is not None:
                self.cap = cap
                self.ok = True
                self.frame = frame
                self.info = f"{frame.shape[1]}x{frame.shape[0]}"
                self.open_ms = (time.time() - t0) * 1000
                self.running = True
                self.thread = threading.Thread(target=self._loop, daemon=True)
                self.thread.start()
                return True
            time.sleep(0.3)
        cap.release()
        self.info = "无画面（可能未授权/被占用）"
        return False

    def _loop(self) -> None:
        t0 = time.time()
        while self.running and self.cap is not None:
            ok, frame = self.cap.read()
            if ok and frame is not None:
                with self.lock:
                    self.frame = frame
                    self.n += 1
                    self.fps = self.n / max(time.time() - t0, 1e-6)

    def latest(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def close(self) -> None:
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        if self.cap is not None:
            self.cap.release()


cams: list[Cam] = []
for i in range(MAX_PROBE):
    c = Cam(i)
    if c.open():
        cams.append(c)
        print(f"[cam {i}] OK  {c.info}  首帧 {c.open_ms:.0f} ms", flush=True)
    else:
        print(f"[cam {i}] --  {c.info}", flush=True)

if not cams:
    print("没有可用的摄像头（检查系统设置→隐私与安全性→摄像头 是否授权）", flush=True)
    err_root = tk.Tk()
    err_root.title("CamView — 无可用摄像头")
    err_root.configure(bg="#111827")
    tk.Label(err_root, text="没有可用的摄像头", bg="#111827", fg="#f87171",
             font=("Helvetica", 22)).pack(padx=44, pady=(34, 8))
    tk.Label(
        err_root,
        text=("可能原因：\n"
              "  1. 摄像头权限未授予\n"
              "     系统设置 → 隐私与安全性 → 摄像头 → 打开本程序（CamView / Terminal）\n"
              "  2. 摄像头被其他程序占用（QuickTime / 相机 / 会议软件）\n"
              "  3. USB 连接或供电问题\n\n"
              "授权后重新打开本程序即可。\n按 Esc 关闭本窗口。"),
        bg="#111827", fg="#e5e7eb", font=("Helvetica", 15), justify="left",
    ).pack(padx=44, pady=(0, 30))
    err_root.bind("<Key>", lambda e: err_root.destroy() if e.keysym == "Escape" else None)
    err_root.focus_force()
    err_root.mainloop()
    sys.exit(1)

print(f"共打开 {len(cams)} 个摄像头: " + ", ".join(f"idx{c.idx}" for c in cams), flush=True)

root = tk.Tk()
root.title(f"摄像头预览 × {len(cams)}")
root.configure(bg="#111827")

tile_w = 640
labels: list[tk.Label] = []
titles: list[tk.Label] = []

for col, c in enumerate(cams):
    t = tk.Label(root, text=f"摄像头 {c.idx}  {c.info}", bg="#111827", fg="#e5e7eb",
                 font=("Helvetica", 14))
    t.grid(row=0, column=col, pady=(8, 2))
    lb = tk.Label(root, bg="#111827")
    lb.grid(row=1, column=col, padx=8, pady=8)
    titles.append(t)
    labels.append(lb)

hint = tk.Label(root, text="S = 保存快照   Q / Esc = 退出", bg="#111827", fg="#9ca3af",
                font=("Helvetica", 13))
hint.grid(row=2, column=0, columnspan=len(cams), pady=(0, 12))


def snapshot() -> None:
    SNAP_DIR.mkdir(parents=True, exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    for c in cams:
        f = c.latest()
        if f is None:
            continue
        p = SNAP_DIR / f"cam{c.idx}_{ts}.png"
        cv2.imwrite(str(p), f)
        print(f"[snapshot] {p}", flush=True)


def on_key(e) -> None:
    if e.keysym in ("Escape", "q", "Q"):
        root.destroy()
    elif e.keysym in ("s", "S"):
        snapshot()


root.bind("<Key>", on_key)
root.focus_force()

photos: list[ImageTk.PhotoImage | None] = [None] * len(cams)


def tick() -> None:
    for i, c in enumerate(cams):
        f = c.latest()
        if f is None:
            continue
        rgb = cv2.cvtColor(f, cv2.COLOR_BGR2RGB)
        im = Image.fromarray(rgb)
        w, h = im.size
        if w != tile_w:
            im = im.resize((tile_w, int(h * tile_w / w)))
        ph = ImageTk.PhotoImage(im)
        photos[i] = ph
        labels[i].configure(image=ph)
        titles[i].configure(text=f"摄像头 {c.idx}  {c.info}  {c.fps:4.1f} fps")
    root.after(40, tick)


root.after(100, tick)
root.mainloop()

for c in cams:
    c.close()
print("[cam-view] 已关闭全部摄像头", flush=True)
