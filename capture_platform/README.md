# rebot-capture · 采集端

> 本地优先、离线可用的采集端：**设备状态 → 遥操 → 录制 → 质检 → 打包 LeRobot 数据集**。
> 设计文档见 `../采集平台方案.md`。

当前是 **v0.1**：全链路已打通（**遥操 → 录制 → 质检 → 打包 → 回放**）。真机后端已接入并在 reBot Arm B601-RS 上实测采集（已产出 4 个数据集、6 条 episode）；无硬件时用 Mock 后端即可跑通自检。

---

## 快速开始

```bash
# 1) 建环境（任选其一）
python3 -m venv .venv                       # 用系统/conda 的 python3.10+
# 或复用已有 conda 环境：conda activate rebot

# 2) 安装（国内建议走清华镜像）
.venv/bin/pip install -U pip -i https://pypi.tuna.tsinghua.edu.cn/simple
.venv/bin/pip install -e ".[camera]" -i https://pypi.tuna.tsinghua.edu.cn/simple

# 3) 自检（不需要硬件）：遥操→录制→质检→打包 全链路
.venv/bin/rebot-capture selftest

# 4) 起采集端服务，浏览器打开 http://127.0.0.1:8787
.venv/bin/rebot-capture serve --backend mock

# 5) 环境体检
.venv/bin/rebot-capture doctor
```

自检预期输出：2 个合格 episode（A/B）+ 1 个顶限位被拦（F），打包只含合格数据，目录校验通过。

---

## 目录结构

```
rebot_capture/
├── configs/rebot_b601_rs.json      # 设备档案（电机ID/型号/限位/增益，来自实测）
├── rebot_capture/
│   ├── device/                     # 设备层：档案 / 后端协议 / Mock 后端 / 真机后端（B601-RS 实测）
│   ├── teleop/                     # 输入层：笔样本 / 映射（真机走 teleop_core，Mock 走 MockPenMapper）
│   ├── recorder/                   # 录制层：episode / 会话 / 落盘 + CaptureService
│   ├── quality/                    # 质量层：硬门规则 + 评分卡（A/B/C/F）
│   ├── packer/                     # 打包层：LeRobot 风格数据集 + 校验
│   ├── server/                     # 服务层：REST + WebSocket + 静态 UI
│   ├── web/                        # 采集端 Web UI（无框架）
│   ├── selftest.py                 # 无硬件自检
│   └── cli.py                      # serve / selftest / doctor
├── docs/API.md                     # 接口契约
└── data/                           # 运行产物（自动创建，可加进 .gitignore）
    ├── datasets/                   # 打包后的数据集
    └── logs/
```

---

## 关键约定

| 项 | 约定 |
|---|---|
| 观测向量 | 7 维 = 6 关节（rad）+ 夹爪（0–1） |
| 动作向量 | 7 维 = 6 关节目标（rad）+ 夹爪目标（0–1） |
| 数据集 | LeRobot 风格：`meta/info.json` + `data/chunk-000/*.parquet` + `meta/rebot_meta.json` |
| 质量门 | 硬门（丢帧<2%、限位<5%、时长、时间戳、成功标记）不过 → 等级 F，禁止打包 |
| 评分 | A ≥ 85 ｜ B ≥ 70 ｜ C < 70 ｜ F（硬门未过） |
| 安全 | 只有 park 位附近允许失能；真机端接状态机（见方案 §5.2） |

---

## 真机模式（reBotArm_control_py）

平台**不复制**上游代码，按路径加载 `reBotArm_control_py`（默认自动探测 `~/Desktop/reBotArm_control_py`，
也可 `--arm-repo` 或环境变量 `REBOT_ARM_REPO`）。

```bash
# 1) 装齐真机依赖（pin / motorbridge / pyyaml）
.venv/bin/pip install -e ".[arm]" -i https://pypi.tuna.tsinghua.edu.cn/simple

# 2) 先做一次无硬件自检（运动学 + 映射 + 安全盒）
.venv/bin/rebot-capture selftest --ik

# 3) 起服务（真机；启动前机械臂需在折叠零位附近 ≤0.05 rad）
.venv/bin/rebot-capture serve --backend rebot --arm-repo ~/Desktop/reBotArm_control_py
```

**已实现的真机行为（= `reBotArm_control_py/teleop_core.py`，与 `spatial_teleop.py` 同逻辑）**

| 能力 | 说明 |
|---|---|
| 控制核心 | `teleop_core.py`（从 `spatial_teleop.py` 的 `control_loop` 抽出，**控制律零改动**） |
| 启动安全门 | 距零位 >0.05 rad 拒绝使能，且**不失能**；+ 等 7/7 电机反馈齐全再判姿态 |
| 控制 | MIT + 重力前馈；速度级：笔位移→末端速度→阻尼最小二乘雅可比→零空间→限幅积分 |
| 速度档 | 慢/中/快 = 0.5/1.0/2.0（4/8/16 cm/s） |
| 冻结 | Space（立即刚性保持） |
| 姿态模式 | O（角速度上限 30°/s） |
| 悬停/漂浮 | H（kp=0 + 重力补偿，可手推） |
| 关节直控 | Q/A 按住 + `[`/`]` 选关节（J1–J6 + 夹爪）；`,`/`.` 控 J6 自转 |
| 预设 | 1–4 前往 / Shift+1–4 记录（最小 jerk 回放） |
| 对齐 | R（重设姿态参考/构型正则/锚点） |
| 安全盒 / 限位 | BX/BY/BZ + RMAX 0.48；关节余量 1.5°；qd ≤ 1.0 rad/s |
| 夹爪 | 直控（J7 + Q/A），kp=20/kd=2/1.2 rad/s/力矩保护 1 N·m 回退 |
| 归零 / 退出 | 最小 jerk（10a³−15a⁴+6a⁵）→ 保持悬停、**不失能**；异常退出先温和归零（>25 N·m 停止） |
| 失能 | 仅 `disable()`（park 位附近），等价上游 `disable_arm.py` |

**平台键位（浏览器内，与上游一致）**

`Space` 冻结 ｜ `F` 速度档 ｜ `O` 位置/姿态 ｜ `H` 悬停 ｜ `R` 对齐 ｜
`Q`/`A` 关节正/反转（按住）｜ `[`/`]` 选关节 ｜ `,`/`.` J6 自转 ｜ `1–4` 预设（`Shift+1–4` 记录）｜ `Esc` 冻结

> 现场建议流程：`goto` ready 位 → 录 episode → 结束 → 打包；退出前务必 `park`
> （或直接关 48V，上游已知"看门狗释放、折叠位自稳"行为）。

---

## 与平台方案对应关系

| 方案章节 | 本仓库落点 |
|---|---|
| §4.3 采集端 App | `server/` + `web/` |
| §5.1 一键配置 | `device/profile.py` → `to_auto_config()` |
| §5.2 安全状态机 | `device/real_arm.py`（启动安全门 / MIT+重力前馈 / 力矩保护 / 归零与失能门，均已实现） |
| §6 数据标准 | `packer/lerobot.py` + `meta/rebot_meta.json` |
| §7 质量门/评分卡 | `quality/rules.py` + `quality/scoring.py` |
| §9 API | `server/app.py`（见 docs/API.md） |
| §12 现有资产复用 | `tablet_teleop` 笔事件 → `/ws/teleop`；`reBotArm_control_py` → 真机后端 |
