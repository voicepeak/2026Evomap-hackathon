# reBot 具身智能数据采集工作区

> 打包日期：2026-09-23
> 内容：**采集平台 + 机械臂控制核心 + 早期遥操实验 + 设计文档 + 已采集数据**
> 目标：一台机器、一个目录，就能继续"遥操作采集 → 质检 → 数据集 → 回放/训练"的闭环。

---

## 一、目录结构

```
rebot_workspace_20260923/
├── README.md                  ← 本文件
├── workspace_manifest.json    ← 各组件清单（文件数/体积/数据索引摘要）
├── capture_platform/          ← 采集平台（RDP 采集端）
│   ├── rebot_capture/         ← 源码：设备/遥操/录制/质检/打包/服务/Web UI
│   ├── tools/camera_bridge.py ← 备用相机桥（一般不需要，见下方"相机权限"）
│   ├── scripts/               ← 启动脚本（run_serve.command / dev.sh / selftest.sh）
│   ├── configs/               ← 设备档案（B601-RS 电机/限位/增益）
│   ├── docs/API.md            ← 接口契约
│   └── data/                  ← ★ 采集数据与清单（见第三节）
├── arm_control/               ← reBotArm_control_py（遥操控制核心，含 .git 历史）
│   ├── teleop_core.py         ← 无 GUI 控制核心（平台调用的就是它）
│   ├── spatial_teleop.py      ← 你原来的完整遥操台（保留，未改动）
│   ├── config/                ← 电机/CAN/MIT 参数、预设姿势
│   ├── tools/                 ← go_zero / disable_arm / 夹爪标定 等工具
│   ├── urdf/ models/          ← RS 版 URDF 与网格
│   └── logs/                  ← 之前的运行日志
├── tablet_teleop/             ← 早期数位板遥操实验（WebSocket 桥 + 网页）
├── site/                      ← 项目站点（GitHub Pages 用，零依赖静态页）
│   ├── index.html / style.css / app.js
│   ├── assets/                ← 截图、腕部相机动图、favicon
│   └── publish.command        ← 发布到 gh-pages 分支（也可走 GitHub Actions）
└── docs/                      ← 设计文档
    ├── 具身智能遥操数据平台_整体方案_v1.0.md
    ├── 采集平台方案.md
    ├── 黑客松Demo_方案.md
    └── tablet_teleop_PLAN.md
```

---

## 二、快速启动

### 0. 环境（一次性）

```bash
cd capture_platform
python3 -m venv .venv                       # Python 3.10+（本机用的是 3.12）
.venv/bin/pip install -e ".[camera,arm,gui]" -i https://pypi.tuna.tsinghua.edu.cn/simple
```

`arm` 额外依赖会装 `pin`（Pinocchio）/`motorbridge`/`pyyaml`；`camera` 会装 `opencv-python-headless`；
`gui` 会装 `PySide6-Essentials`（桌面界面）。

> ⚠️ **本机 venv 的"基础 Python"要留意**：这个 `.venv` 最早是用 Codex 运行时缓存里的 Python 3.12 建的
> （`~/.cache/codex-runtimes/…/python`）。缓存被清理/更新会让整套依赖失效。
> 工作区已保留副本 `.rollback/runtime_python_20260923/`，出问题**双击 `scripts/repair_env.command`** 即可指回来。

### 1. 机械臂归零（每次上电/失能后必做）

**服务在运行时**：点遥操作界面下方中间的「回零」（平滑回零、保持悬停）。

**服务已停止时**（在 Terminal 里跑；工作区 `arm_control` 没有独立 venv，用带依赖的副本）：

```bash
cd /Users/Admin/Desktop/reBotArm_control_py
.venv/bin/python tools/go_zero.py           # 温和归零（会先使能、再 0.25 rad/s 斜坡回零）
```

> ⚠️ 不要同时用：服务运行中再跑 go_zero，两边会抢机械臂控制。

> 平台启动有安全门：**距零位 >0.05 rad 会拒绝使能**（不动、不失能）。所以先归零再起服务。
> 失能只走 `disable_arm.py`（先归零后失能）。

### 2. 启动（推荐桌面 GUI）

```bash
cd capture_platform
./scripts/run_gui.command          # 双击也行：自动拉起服务 + 打开原生界面
```

`run_gui.command` 做了三件事：① 在 Terminal 里拉起采集服务（继承相机权限，已在跑就不重复起）
② 打开原生 GUI ③ `caffeinate` 防止演示期间显示器休眠（休眠会让全屏画面变黑，容易误判成程序挂了）。

- 「面板」按钮 / `Tab` 在 **遥操作视图 ↔ 控制面板** 之间切换（面板里配相机、看关节、打包、回放）
- 键盘 `Space/F/O/H/R/Q/A/[/]/,/.` 与网页版一致（`F11` 全屏，`Q/A` 按住点动）
- 原生全屏在 macOS 上若没生效（后台启动时常见），会自动回退成**无边框铺满 + 置顶**
- GUI 只是客户端：遥操/录制/相机都在服务进程里，**后端逻辑没动**；网页界面仍可兜底使用

<details>
<summary>网页界面（备用 / 远程看画面）</summary>

```bash
cd capture_platform
./scripts/run_serve.command                 # 或：.venv/bin/rebot-capture serve --backend rebot --arm-repo ../arm_control
```

打开 **http://127.0.0.1:8787** → 相机卡片选 `wrist` → 开始会话 → 开始 Episode → 遥操 → 结束（成功）→ 打包数据集。
服务日志写在 `/tmp/rebot_workspace_serve.log`。

</details>

**为什么必须在 Terminal 里启动**：macOS 相机权限是按 App 授权的，`OpenCode.app` 没有相机权限，而 `Terminal.app` 有。
在 Terminal 里启动 → 服务进程继承 Terminal 的权限 → 摄像头可直接打开。

**演示前请关掉旧浏览器标签页**：任何指向 `127.0.0.1:8787` 的旧页面都会往同一个服务发笔样本
（实测见过一个标签页因 `pointerup` 丢失而持续发"落笔"，笔通道一直不是 0Hz，相当于有人一直按着笔）。

### 3. 笔操作（GUI 与网页一致，可全程不用键盘）

- **笔尖落笔**：位置模式 = 前后/横移；姿态模式 = 俯仰/摆头
- **笔右键**（多数数位板在 macOS 上是"鼠标模拟"，只用右键做切换；不区分悬空/落笔）：
  - **单击**（松开时触发）= 循环选择电机（J1…J6、夹爪）
  - **长按**（按住约 0.5s）= 切换 位置/姿态
- **电机直控**（选中电机后）：按该电机的功能象限驱动——俯仰类（J2/J3/J4）用笔上下，回转类（J1/J5/J6）用笔左右；
  选中**夹爪**时为笔上划 = 张开、下划 = 闭合（手动兜底）
- **速度手感**（2026-09 调）：所有速度指令（位置/姿态/电机直控/Q·A）先过
  "限加速 + 30ms 低通"再下发，起停与换向不再一帧内 0↔满速（原来单电机直控会明显"窜一下"）；
  电机直控另有 **PID 速度环**（用电机反馈的实测角速度闭环），摩擦/负载造成的速度差会被补回来，
  所以是"更顺"而不是"更慢"——满偏速度不变（关节 60°/s、末端 8cm/s×速度档，姿态 40°/s×速度档）。
  参数都在 `arm_control/teleop_core.py` 的 `CoreArgs`（`v_acc/v_tau/w_acc/w_tau/j_acc/j_tau`、
  `jvel_kp/ki/kd`、`dead/expo/wmax_deg`），调完不用改别的代码
- **手势夹爪**（电脑摄像头）：**张开手 = 夹爪张开行程**，**握拳 = 夹爪闭合行程**，不区分左右手；
  同一手势保持不会反复触发，手离开后再做同一手势会重新生效（手动调整过夹爪后不会被抢回）；
  电机被保护性失能时平台会自动重新使能（自愈）。默认相机 `--gesture-camera 2`，界面左上按钮可临时开关。
  全屏界面右上角显示两路小画面：腕部相机 + 电脑摄像头。

---

## 三、数据（★ 最值钱的部分）

位置：`capture_platform/data/`

```
data/
├── MANIFEST.json              ← 清单 + 每条 sha256（防篡改/防丢）
├── ARCHIVE_20260923.tar.gz    ← 全部原始数据归档
├── episodes/                  ← 每条 episode 的原始 parquet（录完自动落盘）
└── datasets/                  ← 打包后的数据集（LeRobot 风格）
    ├── session_0905_all/b        3 集（含 2 条 F 级留档）
    ├── session_2/                2 集（A + B）
    └── session_3/                1 集（B）
```

- **自动落盘**：每条 episode 结束即写 `episodes/*.parquet`（不管等级，F 级也留）
- **历史可见**：面板「历史数据（磁盘）」列出已打包数据集（含每条等级/质量分，可点回放）和原始留档；
  接口 `GET /api/library`。会话内的 episode 列表则在服务重启后会清空（内存态），历史数据不受影响
- **打包**：只有通过质量硬门（丢帧<2%、限位<5%、有时长、成功标记）的才会进 `datasets/`；
  若全部不合格，界面会询问**是否包含未过质检的 episode 一起打包**（等级如实写进元数据），
  也可以直接调 API：`-d '{"task_instruction":"grasp and place","include_failed":true}'`（数据集名字需换新的）
- **视频**（接相机后）：`datasets/<name>/videos/chunk-000/observation.images.<相机名>/episode_XXXXXX.mp4`
- 清单重建：`cd capture_platform && .venv/bin/rebot-capture index`

---

## 四、回放（演示/复现）

**界面里两种来源**：

- 面板「历史数据（磁盘）」里点任意一条（如 `#1 A 85.6 ▸`）→ 回放**磁盘上已打包数据集**里的那条，
  服务重启也不影响（走 `dataset_path` + `index`）
- 面板「录制 / 数据集」里的 `1× 2× 4×` → 回放**本次服务进程内**刚录的 episode

```bash
# 命令行等价写法：从数据集回放第 2 条
curl -X POST http://127.0.0.1:8787/api/replay \
  -H 'Content-Type: application/json' \
  -d '{"dataset_path":"data/datasets/session_3","index":2,"speed":4.0,"max_joint_speed_deg":720,"tau_abort":25}'
```

回放中：`Space` 冻结中止｜落笔接管｜力矩超限自动停。回放前会先用最小 jerk 曲线到轨迹起点。

历史数据清单（界面那块卡片用的接口）：

```bash
curl -s http://127.0.0.1:8787/api/library | python3 -m json.tool | head -30
```

---

## 五、安全约定（都已在代码里）

1. **启动安全门**：非零位拒绝使能（不失能）
2. **所有走位**走最小 jerk 曲线（10a³−15a⁴+6a⁵），不是阶跃
3. **退出/Ctrl+C**：先平滑归零再退出，**不失能**
4. **失能**只在 park 位附近允许（`disable_arm.py`）
5. **异常退出**：温和归零 + 力矩 >25 N·m 立即停止、原地保持
6. **回放/走位**：力矩超限自动中止并保持
7. **遥操**：抬笔即停（0.4s 输入超时；速度指令带 30ms 低通 + 加速度限幅，起停/换向是连续斜坡而不是瞬间跳变，松手约 0.1s 内停稳）、安全盒 BX/BY/BZ+RMAX 0.48（**软边界**：接近边界按剩余空间成比例减速，可以慢慢贴边但不会越界，无"推不动"死区）、关节软限位 1.5°（限位排斥已减弱为 0.12 rad/s / 3° 作用带，避免限位反弹）、QD ≤1.0 rad/s

---

## 六、已知限制 / 待办

| 项 | 说明 |
|---|---|
| 相机权限 | 平台必须在 Terminal 启动；`OpenCode.app` 授权需重启 App 才生效 |
| 单相机录制 | 当前只录"选中的那一路"；**多路同录（wrist+scene）待加** |
| 控制环频率 | 默认 100Hz（上游 200Hz）；`--fps 200` 可提速，回放倍速会更接近标称 |
| 数据规模 | 6 条 / ~4.6 分钟；训练需 ≥50 条/任务 |
| LeRobot 转换 | 数据集是 LeRobot 风格但未逐字段对齐 v3；需转换器 |
| 自动成功判定 | 现在靠人工按"成功"；需接入夹爪力/位置判定 |
| 溯源 | MANIFEST 有哈希；设备序列号/标定/授权字段待补 |
| 限位占用门 | J2/J3 的**零位就是下限**，停在折叠零位会被算成"限位占用"→ 可能判 F（演示前彩排确认，必要时用上面的 `include_failed`） |
| 回放来源 | 界面上的 1×/2×/4× 用**内存里最近的 episode**；服务重启后需先录一条或先打包 |

---

## 七、组件速查

| 组件 | 关键文件 | 作用 |
|---|---|---|
| 采集平台 | `capture_platform/rebot_capture/server/app.py` | REST + WS + Web UI |
| **桌面 GUI** | `capture_platform/rebot_capture/gui/` | PySide6 原生界面（服务客户端） |
| 控制核心（上游逻辑） | `arm_control/teleop_core.py` | 速度档/冻结/姿态模式/漂浮/直控/预设 + 速度整形与直控 PID 速度环 |
| 平台适配器 | `capture_platform/rebot_capture/teleop/rebot_core.py` | 把 teleop_core 接进平台（笔坐标 960×620 虚拟窗口） |
| 真机后端 | `capture_platform/rebot_capture/device/real_arm.py` | 连接/读状态/MIT 下发/归零/失能门 |
| 质量门 | `capture_platform/rebot_capture/quality/` | 硬门 + A/B/C/F 评分 |
| 打包器 | `capture_platform/rebot_capture/packer/lerobot.py` | 数据集目录 + 视频 + 元数据 |
| 相机 | `capture_platform/rebot_capture/camera.py` | 预览/录制/命名（wrist、scene） |
