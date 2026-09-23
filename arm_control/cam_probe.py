#!/usr/bin/env python3
"""cam_probe.py — 依次尝试摄像头编号 0..3，报告哪个能开、分辨率多少。

用法（在 Terminal 里跑，需要摄像头权限）:
    ~/.local/bin/uv run python cam_probe.py 2>&1 | tee logs/cam_probe.log
"""
import time

import cv2

print("=== 摄像头探测（0..3）===", flush=True)
found = []
for i in range(4):
    cap = cv2.VideoCapture(i, cv2.CAP_AVFOUNDATION)
    if not cap.isOpened():
        cap.release()
        cap = cv2.VideoCapture(i)
    if not cap.isOpened():
        print(f"index {i}: ❌ 打不开（不存在 / 被占用 / 未授权）", flush=True)
        continue
    ok, frame = cap.read()
    if ok and frame is not None:
        h, w = frame.shape[:2]
        print(f"index {i}: ✅ 可用  {w}x{h}", flush=True)
        found.append(i)
    else:
        # 再等一会儿试试（有些摄像头启动慢）
        for _ in range(5):
            time.sleep(0.3)
            ok, frame = cap.read()
            if ok and frame is not None:
                break
        if ok and frame is not None:
            h, w = frame.shape[:2]
            print(f"index {i}: ✅ 可用（慢启动）  {w}x{h}", flush=True)
            found.append(i)
        else:
            print(f"index {i}: ⚠️ 能打开但读不到画面（可能被其他 App 占用 / 虚拟摄像头）", flush=True)
    cap.release()

print()
if found:
    print("可用的编号: " + ", ".join(str(i) for i in found), flush=True)
    print(f"建议命令: ~/.local/bin/uv run python cam_server.py --index {found[0]}", flush=True)
else:
    print("没有可用摄像头：检查 系统设置→隐私与安全性→摄像头 是否授权 Terminal；"
          "并关掉可能占用摄像头的 App（Chrome / 微信 / 会议软件 / 相机）", flush=True)
print("=== 探测结束 ===", flush=True)
