# reBot 具身智能数据采集平台（2026 Evomap Hackathon）

> **把"用机械臂采数据"做成一条流水线**：数位板遥操作 → 实时质检 → 标准数据集 → 1×/2×/4× 回放。
> 硬件：Seeed **reBot Arm B601-RS**（6+1 DOF，RobStride RS06×3 + RS00×4，CAN 1Mbps，48V）

---

## ✨ 亮点（全部在真机上验证过）

| 能力 | 说明 |
|---|---|
| **数位板遥操** | 笔位移 → 末端速度 → 阻尼最小二乘雅可比 → 零空间（限位排斥/构型正则）→ 软限位硬夹；抬笔即停（0.4s 输入超时） |
| **上游同源逻辑** | 控制律直接复用 `arm_control/teleop_core.py`（从 `spatial_teleop.py` 抽出，**零改动**）：速度档 4/8/16cm/s、冻结、姿态模式、悬停漂浮、关节直控、姿势预设 |
| **安全体系** | 启动安全门（非零位拒绝使能且不失能）、**全程最小 jerk 走位/归零**（10a³−15a⁴+6a⁵）、退出不失能、力矩保护（>25 N·m 停止）、笛卡尔安全盒 + 关节软限位 1.5° |
| **数据流水线** | 逐 episode **自动落盘** → 7 条质量硬门 → A/B/C/F 评分 → LeRobot 风格数据集（含视频） |
| **回放** | 1×/2×/4× 倍速（带关节速度上限保护）；可从内存或已打包数据集回放；回放中按 `Space` 冻结中止、落笔接管 |
| **多相机** | wrist / scene 双路预览与录制（macOS 权限方案：服务在 Terminal 启动） |
| **数据清单** | `MANIFEST.json` 逐文件 sha256，防丢防篡改 |

---

## 📁 目录结构

```
.
├── docs/                     设计文档（整体方案 / 采集平台方案 / Demo 方案 / 数位板方案）
├── arm_control/              reBotArm 控制核心（107MB，含 URDF 数字孪生网格）
│   ├── teleop_core.py        ★ 无 GUI 控制核心（平台调用的就是它）
│   ├── spatial_teleop.py     完整遥操台（上游原样保留）
│   ├── tools/                go_zero（温和归零）/ disable_arm（安全失能）/ 夹爪标定
│   ├── config/               电机 ID / 型号 / MIT 增益 / 预设姿势
│   └── urdf/                 RS + DM 模型与网格
├── capture_platform/         采集平台
│   ├── rebot_capture/        device / teleop / recorder / quality / packer / server / web
│   ├── configs/              B601-RS 设备档案（限位/增益）
│   ├── tools/ scripts/       相机桥、启动脚本
│   ├── docs/API.md           接口契约（REST + WS）
│   └── data/                 ★ 采集数据（数据集 / 原始 episode / 清单）
└── tablet_teleop/            早期数位板遥操实验（WebSocket 桥 + 网页）
```

---

## 🚀 快速开始

```bash
# 0) 环境
cd capture_platform
python3 -m venv .venv && .venv/bin/pip install -e ".[camera,arm]" -i https://pypi.tuna.tsinghua.edu.cn/simple

# 1) 机械臂归零（每次上电/失能后）
cd ../arm_control && .venv/bin/python tools/go_zero.py

# 2) 启动平台（★ 必须在 Terminal 里启动：macOS 相机权限按 App 授权）
cd ../capture_platform && ./scripts/run_serve.command

# 3) 浏览器 → http://127.0.0.1:8787
#    相机卡片选 wrist → 开始会话 → 开始 Episode → 遥操 → 结束（成功）→ 打包数据集
```

**键位（与上游 `spatial_teleop.py` 一致）**
`Space` 冻结 ｜ `F` 速度档 ｜ `O` 位置/姿态 ｜ `H` 悬停 ｜ `R` 对齐 ｜
`Q/A` 关节直控（按住）+ `[`/`]` 选关节 ｜ `,`/`.` J6 自转 ｜ `1–4` 预设（`Shift+1-4` 记录）

**回放示例**
```bash
curl -X POST http://127.0.0.1:8787/api/replay -H 'Content-Type: application/json' \
  -d '{"dataset_path":"capture_platform/data/datasets/session_3","index":2,"speed":4.0,
       "max_joint_speed_deg":720,"tau_abort":25}'
```

---

## 📊 数据现状

| 数据集 | 内容 |
|---|---|
| `session_0905_all` | 3 条（含 2 条 F 级留档） |
| `session_0905_b` | 1 条（105.9s，B 级） |
| `session_2` | 2 条（5.5s A 级 + 57.0s B 级） |
| `session_3` | 1 条（59.4s B 级） |

**合计：6 条 episode / 25,458 帧 / ≈4.6 分钟**（清单与哈希见 `capture_platform/data/MANIFEST.json`）

数据通道：关节位置（实测）+ 动作（指令目标）+ 夹爪 + 笔压力/接触 + 时间戳 + 速度/力矩（新录制）；接相机后含 `observation.images.wrist` 视频。

---

## ⚠️ 已知限制 & 下一步

| 项 | 状态 |
|---|---|
| 数据规模 | 6 条 ≈ **工具链验证集**；训练需 ≥50 条/任务（含视觉） |
| 双路同录 | 当前只录选中相机；wrist + scene 同录待加 |
| LeRobot 精确对齐 | 目录结构对齐 v3，字段/统计需转换器 |
| 自动成功判定 | 现为人工标记；需夹爪力/位置判定 |
| 控制环频率 | 默认 100Hz（上游 200Hz），`--fps 200` 可提速 |
| 溯源 | MANIFEST 有哈希；设备序列号/标定/授权字段待补 |

---

## 🙏 来源与许可

- 机械臂控制核心：[`reBotArm_control_py`](https://github.com/Seeed-Projects/reBotArm_control_py)（Seeed reBot Arm B601-RS）——本仓库保留原样，新增 `teleop_core.py` 供平台复用
- 数据格式：对齐 Hugging Face **LeRobot** 数据集结构
- 电机 SDK：[motorbridge](https://pypi.org/project/motorbridge/)（RobStride / Damiao 等）
