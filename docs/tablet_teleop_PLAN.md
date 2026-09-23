# 数位板控制机械臂 —— 方案 v0.1

目标：用数位板（x, y, 压力, 倾角）实时控制 reBot Arm B601-RS 完成**平面接触类任务**
（擦拭、按压、涂抹、书写），核心是把**笔压映射成法向接触力**并闭环。

适用硬件：reBot Arm B601-RS（6+1 DOF，RobStride QDD，CAN 1Mbps，MIT 模式）

---

## 0. 设计原则

1. **笔压只映射到"力"，不映射到 Z 位置。** 位置由力闭环自己解算 —— 这样它自动适应未知的
   桌面高度和海绵软硬，绕开"操作者感觉不到真实表面"这个死穴。
2. **切向走阻抗，法向走力。** 这是接触类任务的标准做法（hybrid force/position）。
3. **所有安全限制在软件里做双份**（映射层 + 下发层）。
4. **不改动 `reBotArm_control_py` 的公共 API**，只在外部新建模块。

---

## 0.5 硬件与环境前置 ⚠️ 先做这个

### 平台限制（硬性的）

`config/rebotarm.yaml` → `hardware_yaml: rebotarm_rs.yaml` → `channel: can0`。

`actuator/rebotarm.py` 的判断：

```python
if self._channel.startswith("/dev/tty"):
    return Controller.from_dm_serial(...)   # 串口适配器
return Controller(self._channel)            # SocketCAN —— 只有 Linux 有
```

**`can0` 是 SocketCAN，macOS 上不存在。** 官方 wiki 也明确说：

- "已验证**虚拟机的性能不足以支撑 Demo 运行**且存在配置问题，建议优先使用 **Ubuntu 物理机**"
- "如果您在 macOS 上遥操时**帧率偏低**，可能是 CH34x 驱动版本过旧导致"

本方案是 **200Hz 力控闭环**，比官方 demo 对实时性更敏感。帧率掉到 50Hz，
导纳控制会抖、力会过冲、力估计会被噪声淹没。

**结论：这个仓库的 RS 版本在 macOS 上跑不了。**

### macOS 上的具体障碍

| 配置 | channel | transport | macOS |
|---|---|---|---|
| `rebotarm_rs.yaml`（你的） | `can0` | SocketCAN | ❌ 不存在 |
| `rebotarm_dm.yaml` | `/dev/ttyACM0` | dm-serial（**Damiao 专用**） | ⚠️ 不是通用 CAN 桥 |

而且 `_make_controller()` 里 `vendor` 参数**根本没被使用** —— 任何 `/dev/tty*` 都会走
`Controller.from_dm_serial()`，那是达妙电机的串口协议，不是 RobStride 的 CAN。

**所以第一件事：搞清楚 motorbridge 在 macOS 上怎么连 RobStride。**

```bash
ls /dev/tty.*                    # 看适配器叫什么名字
python -c "import motorbridge; print([n for n in dir(motorbridge.Controller) if not n.startswith('_')])"
pip show motorbridge             # 版本
```

**最快的判据**：先试官方的 **MotorBridge Studio** 网页工具（wiki 说支持 macOS）。
如果 Studio 能驱动机械臂 → macOS 通路存在，问题只是代码里该调哪个构造器。
如果 Studio 也不行 → macOS 这条路堵死，直接上 Linux 小盒子。

### 推荐架构：给 Mac 配一个「机械臂服务器」

```
   Mac（你在用）                          Linux 小盒子（N100 / 树莓派）
┌──────────────────────┐               ┌──────────────────────────┐
│  浏览器：数位板采集    │  TCP/WiFi     │  200Hz MIT 控制循环        │
│  可视化 / 力曲线      │ ────────────> │  阻抗 + 导纳 + 力估计      │
│  标定界面             │  50~100Hz     │  CAN → B601-RS            │
│  代码编辑             │ <──────────── │  状态/力 回传              │
└──────────────────────┘               └──────────────────────────┘
```

**为什么这样反而更好**：实时环靠近硬件，网络抖动只影响**设定值**（慢变量），
不影响**内环**。数位板在 Mac 上（浏览器原生支持，体验最好），力控在小盒子上（稳）。

成本：N100 迷你主机 800~1200 元，或树莓派 5 + USB-CAN 约 800 元。

**但先别买。先按 M-1 测。**

### 环境准备清单

| 步骤 | 命令 / 操作 |
|---|---|
| 系统 | Ubuntu 24.04 LTS 物理机（不要虚拟机） |
| Python | 项目已用 `uv`（有 `.venv` / `uv.lock`），沿用即可 |
| 依赖 | `pip install motorbridge` |
| CAN 速率 | `sudo modprobe peak_usb` → `ip -br link` → `sudo ip link set can0 type can bitrate 1000000` → `sudo ip link set can0 up` |
| 零点校准 | MotorBridge Studio（`motorbridge-gateway --bind 127.0.0.1:9002`）→ 1~7 号关节在线 → 校零 |
| 电机参数 | 页面里套用 **`rebot-arm-robstride` 默认模板**，回读校验一致 |
| 安全 | 调试时保持 ≥1 米距离；**禁止热插拔**（拔 XT30 前必须断电） |

### M-1：实测控制率（写任何业务代码之前）

先跑 `example/9_gravity_compensation.py`，能用手掰动机械臂 = 通路正常。

然后在控制循环里插桩，打印 `dt` 的分位数：

```python
# 在 _loop_cb 或自己的循环里
self._t_prev = getattr(self, "_t_prev", None)
t = time.perf_counter()
if self._t_prev is not None:
    self._dt_hist.append(t - self._t_prev)
self._t_prev = t
# 每 2 秒打印一次
a = np.array(self._dt_hist)
print(f"rate={1/a.mean():.0f}Hz  p50={1/np.percentile(a,50):.0f}  "
      f"p99={1/np.percentile(a,99):.0f}  抖动={a.std()*1e3:.2f}ms")
```

**判据：**

| p50 实际控制率 | 结论 |
|---|---|
| ≥ 200 Hz，抖动 < 1 ms | ✅ 直接上力控 |
| 100~200 Hz | ⚠️ 可行，但导纳增益要调小、力要重滤波 |
| < 100 Hz | ❌ 先解决平台/通信，别写业务逻辑 |

配置里 `rate: 500` 是**标称值，不能信**。每个周期里的
`get_positions()` 会向总线发显式反馈请求帧，这是主要开销。

**注意**：这条也是"最大翻车风险"（见 §9.2），M-1 就把结论拿到手。

---

## 1. 系统架构

```
┌──────────────┐   PointerEvent (200Hz+)   ┌─────────────────┐
│  数位板       │ ─────────────────────────>│  浏览器 (localhost) │
│  (Wacom 等)  │  u, v, pressure, tilt      │  采集 + 可视化 + UI │
└──────────────┘                            └────────┬────────┘
                                                     │ WebSocket (JSON, ~200Hz)
                                                     ▼
                                          ┌──────────────────────┐
                                          │  Python 主控进程       │
                                          │  1. TabletState 环形缓冲│
                                          │  2. 通道映射 → TaskCmd │
                                          │  3. 状态机             │
                                          │  4. 安全裁剪           │
                                          │  5. 控制律 (τ_cmd)     │
                                          └──────────┬───────────┘
                                                     │ 200Hz MIT
                                                     ▼
                                          ┌──────────────────────┐
                                          │  RebotArm (arm 组)    │
                                          │  send_mit(pos,vel,kp, │
                                          │           kd,tau)     │
                                          └──────────┬───────────┘
                                                     │ 反馈帧 τ_m
                                                     ▼
                                          力估计 → 回传浏览器画曲线
```

**为什么用浏览器采数**：`PointerEvent` 原生就有 `pressure / tiltX / tiltY / twist /
altitudeAngle / azimuthAngle`，跨平台、不需要装驱动，而且顺便白送一个可视化和标定界面。

**采样率陷阱**：`pointermove` 被浏览器节流到帧率（60~120Hz）。必须用
`event.getCoalescedEvents()` 取出两次渲染之间**全部**的笔点，才能拿到数位板真实的 200Hz+。

---

## 2. 通道映射

| 数位板通道 | 范围 | 映射到 | 说明 |
|---|---|---|---|
| `u, v`（笔在板上的位置） | 0~1 | 工作台平面上的 (X, Y) | 增益可调，默认 1:1 |
| `pressure` | 0~1 | **法向目标力 `F_des`** | `F_des = F_min + p·(F_max−F_min)` |
| `altitudeAngle` / `tiltX,tiltY` | | 工具姿态（可选） | v1 固定垂直，暂不用 |
| `twist` | | 工具绕自身轴旋转（可选） | v1 不用 |
| 落笔（pressure 0→>0） | | 进入接触 | 状态机 |
| 抬笔（pressure → 0） | | 抬起到安全高度 | 状态机 |
| 笔杆按钮 | | 离合器（暂停跟随，便于重新定位） | 可选 |
| 离开感应区 | | 触发看门狗 → 抬起 | |

**v1 固定工具姿态垂直于工作台**，只动位置。姿态用一次 IK 求出来后保持不变 ——
简单、稳定、够用。

---

## 3. 坐标标定

### 3.1 粗标定（先跑通用）

板面和工作台都是矩形，只需要 4 个参数：缩放 `sx, sy`、旋转 `θ`、平移 `tx, ty`。

```yaml
board_to_work:
  scale: [1.0, 1.0]        # 1.0 = 一比一
  rotation_deg: 0.0
  offset_m: [0.0, 0.0]
  work_z: 0.02             # 工作台表面在 base frame 里的高度（大概值）
```

### 3.2 精标定（四点探测）

让机械臂自己去"探"工作台上贴的四个标记点：

1. 人在数位板上依次点四个角，记下 `(u_i, v_i)`
2. 机械臂带着探针到该点上方，**缓慢下压直到力上跳**（`F_n > 阈值`），
   记录接触瞬间的 `(x_i, y_i, z_i)`
3. 用四组对应点拟合仿射变换 + 平面方程

好处：不需要相机，不需要手眼标定，而且**顺便标定了力的接触阈值**。

---

## 4. 控制律

每个控制周期（`rebotarm.rate`，建议 200Hz）执行一遍。

### 4.1 读状态 + 力估计

```python
q, _, tau_m = arm.get_state()           # ⚠️ 不要用返回的 vel！见下
dq = (q - q_prev) / dt                  # 位置差分求速度（官方文档要求）
q_pad = pad_q_for_model(model, q[:6], 6)
tau_g = compute_generalized_gravity(model, q_pad, data)[:6]   # 已有

tau_ext = tau_m[:6] - tau_g - tau_fric(dq)      # 外部力矩（准静态近似）
J = frame_jacobian(q_pad)                        # 6x6，需自己补一个 helper
F_ext = np.linalg.pinv(J.T) @ tau_ext            # 6D wrench
F_n = lowpass(F_ext[:3] @ n_hat, fc=20.0)        # 法向分力（N）
```

**⚠️ 速度必须用位置差分算。** 官方文档写明：
`mechVel (0x701A) 在这版固件上不是 rad/s`，实测差 **2.6~4.8 倍**，而且**符号不一致**。
直接拿 `vel` 做阻尼项会让控制器发散。

**精度预期（重要修正）**：

`docs/gravity_calibration_rs_2026-07-17.md` 里已经实测过了：

| 项 | 实测值 |
|---|---|
| URDF 重力模型精度 | **5~11%**（很好，不需要额外补零偏） |
| **库仑摩擦** | **每关节 0.2~0.5 N·m**，手腕上"和重力本身相当甚至更大" |

换算到末端：**摩擦贡献的力误差约 ±2 N，而且是方向相关的（换向时会跳变）。**

| 任务 | 需要的力 | 靠关节力矩估计行不行 |
|---|---|---|
| 擦白板 / 擦拭 | 5~20 N | ⚠️ 能用，但环路会有滞后 |
| 抓鸡蛋 / 葡萄 | 2~5 N | ❌ **不行** |

**两条对策（建议都做）：**

1. **摩擦前馈**：把实测的 `Fc` 代进 `tau_fric = Fv·dq + Fc·sign(dq)` 减掉。已有数字：
   `j2=0.53, j3=0.49, j4=0.30, j5=0.21` N·m
2. **加一个梁式称重传感器**（约 20 元，HX711 读数）装在手爪和工具之间。
   直接测法向力，绕开整个摩擦问题。**考虑到上面的数字，这是强烈推荐，不是可选。**

代码里把力源做成可插拔的，两种都实现，跑起来对比一次就知道了。

### 4.2 状态机

```
IDLE ──落笔(p>0)──> APPROACH ──接触(F_n>0.5N)──> CONTACT
  ▲                                                │
  └── RETRACT(抬到 z_safe + 横向平移) <──抬笔(p==0)─┘
```

- `APPROACH`：从 `z_safe` 以恒定低速下降，**用一个小 kp 让它轻轻贴住**
- `CONTACT`：法向切到导纳 + 力前馈
- `RETRACT`：抬到 `z_safe`，横向移动时有速度上限，防止甩出去
- 看门狗：超过 `watchdog_ms` 没收到笔数据 → 抬起 + 停

### 4.3 核心：切向阻抗 + 法向导纳

```python
# ---- 目标 ----
p_xy_des = board_to_work(pen.u, pen.v)
F_des    = F_min + pen.pressure * (F_max - F_min)

# ---- 法向：导纳积分（心脏） ----
z_cmd += LAMBDA * (F_des - F_n) * dt          # 力不够就继续往下走
z_cmd  = clip(z_cmd, z_floor, z_ceiling)      # 硬限位，防止压穿

p_des = np.array([p_xy_des[0], p_xy_des[1], z_cmd])

# ---- 笛卡尔阻抗 ----
F_cmd = K_p * (p_des - p_cur) - D_p * v_cur   # N（3 维）

# ---- 合成关节力矩 ----
tau_cmd = tau_g + J[:3, :].T @ F_cmd
```

`LAMBDA` 单位是 **m/(N·s)**。想要"5 N 的力误差在 10 ms 内推动 1 mm"：
`0.001 = LAMBDA × 5 × 0.01` → `LAMBDA ≈ 0.02`。

**为什么这样能工作**：刚性表面上笔压不下去 → 位置误差累积 → 力上升 →
`F_n` 追上 `F_des` → 积分停止。桌高、海绵软硬全都自动适应。

### 4.4 下发

```python
arm.arm.send_mit(
    pos=q_target,           # 由当前位置 + 小步长推进，或用 IK 从 p_des 求
    vel=np.zeros(6),
    kp=kp_c, kd=kd_c,       # 柔顺增益，比默认小
    tau=tau_cmd,
)
```

注：MIT 模式下 `kp` 就是"刚度"。擦拭时 `kp` 建议 **5~15**（不是配置默认的 50）。

---

## 5. 安全

| 项 | 做法 |
|---|---|
| 工作空间 | 目标点先裁到 **70% 臂展（≈0.53 m）** 内；超界在界面上标红，不静默裁剪 |
| 速度 | 关节速度上限 + 笛卡尔速度上限（`max_cart_vel`） |
| 法向力 | `F_max` 软限 + `hard_limit` 硬限，两层 |
| 力矩 | 每关节力矩上限（`tau_clip`） |
| 看门狗 | `watchdog_ms` 无笔数据 → 抬起 + 停 |
| 急停 | 键盘 Esc + `rebotarm.estop()`；物理急停独立 |
| 零点 | 每次上电必须重新标零（官方要求） |
| J2 堵转 | **严格待在 70% 臂展内**，否则 J2 堵转保护 → 掉臂 |

---

## 6. 代码结构

```
tablet_teleop/
├── config/
│   └── tablet.yaml          # 标定、映射、增益、限幅（不动原 SDK 配置）
├── web/
│   ├── index.html           # 数位板采集 + 力曲线 + 工作空间可视化
│   └── app.js               # PointerEvent + getCoalescedEvents + WebSocket
├── bridge.py                # WebSocket 服务器，写进 TabletState 环形缓冲
├── teleop.py                # 主程序：控制循环 + 状态机
├── mapper.py                # PenState -> TaskCmd
├── controller.py            # 阻抗 + 导纳控制律
├── force.py                 # 力估计（可插拔：关节力矩 / 称重传感器）
├── safety.py                # 裁剪、限幅、看门狗
└── kinematics_ext.py        # 补一个 frame_jacobian()（Pinocchio）
```

关键接口：

```python
@dataclass
class PenState:
    u: float; v: float          # 0~1
    pressure: float             # 0~1
    tilt_x: float; tilt_y: float
    altitude: float; azimuth: float; twist: float
    touching: bool
    t: float                    # 单调时间戳

@dataclass
class TaskCmd:
    p_xy: np.ndarray            # 工作台平面上的 XY (m)
    F_n: float                  # 法向目标力 (N)
    contact: bool
```

接入现有 SDK 的两个点：

| 已有 | 用途 |
|---|---|
| `RebotArm` / `JointGroup.send_mit` | 下发 |
| `RebotArm.get_state()` → `(pos, vel, torq)` | 读反馈（torq 就是力估计的原料） |
| `GravityCompensation._gravity_comp_torque()` 的算法 | `tau_g` |
| `load_robot_model` / `compute_fk` / `pad_q_for_model` | 运动学 |
| `solve_ik` / `pos_rot_to_se3` | 姿态求解、`p_des` 求关节角 |
| `load_gravity_compensation_config` / `joint_direction` / `tau_scale` | 对齐模型与真机 |

**要自己补的只有一样**：`frame_jacobian(q)`。Pinocchio 里是
`computeJointJacobians` + `getFrameJacobian(..., ReferenceFrame.LOCAL_WORLD_ALIGNED)`。

---

## 7. 里程碑

| 阶段 | 内容 | 能否演示 |
|---|---|---|
| **M-1** | **实测控制率**（跑通 example 9，插桩测 dt 分位数） | ❌ 但必须先做 |
| **M0** | 数位板 → 网页 → 力曲线 + 轨迹可视化（**不接臂**） | ✅ 纯软件，1 天 |
| **M1** | 网页 → WebSocket → Python → 关节空间跟随（悬空不接触） | ✅ |
| **M2** | 加 IK + 平面映射，悬空画轨迹（1:1） | ✅ |
| **M3** | 加接触：落笔下降，小 kp 轻贴 | ✅ |
| **M4** | 加力闭环：导纳控制（核心） | ✅ |
| **M5** | 状态机 + 抬落笔 + 安全限幅 | ✅ |
| **M6** | 实测标定（擦白板 / 压面团），调 F 范围 | ✅ **可展示** |

**M0 是降级方案**：即使机械臂全崩了，这块屏幕也能讲完整个故事。

---

## 8. 待定参数（需要实测）

| 参数 | 建议初值 | SDK 配置现值 | 说明 |
|---|---|---|---|
| `F_min / F_max` | 0 / 12 N | — | 先保守，擦白板从 5 N 起 |
| `F_hard_limit` | 20 N | — | |
| `K_p` | 300~800 N/m | — | 太大 = 撞击，太小 = 跟不上 |
| `D_p` | 10~30 N·s/m | — | 抑制抖动 |
| `LAMBDA` | 0.02 m/(N·s) | — | 导纳增益 |
| **MIT `kp`** | **5~10（力控时覆盖）** | **J1=50, J2/J3=150, J4~J6=50** | **默认太硬，必须在 `send_mit` 里运行时覆盖** |
| **MIT `kd`** | 1~3 | J1=3, J2/J3=10, J4~J6=4~5 | 同样运行时覆盖 |
| 重力补偿 `tau_scale` | 直接用 | `[1.0, 0.98, 0.98, 1, 1, 1]` | **已标定过，直接复用** |
| 重力补偿 `kp` | 5~15 | `profiles.basic: kp=15, kd=2.5` | 手掰用 15；力控可再降 |
| `z_floor / z_ceiling` | 工作台面 ±5 mm | — | 硬限位 |
| `max_cart_vel` | 0.25 m/s | — | |
| `watchdog_ms` | 200 ms | — | |
| 控制率 | 200 Hz | 标称 `rate: 500` | **必须实测，见 §0.5** |
| 末端帧 | — | `gripper_end` | URDF: `urdf/RS/urdf/ReBot_Arm_RS.urdf` |

---

## 9. 已知风险

1. **力估计精度 ~±2 N**（受库仑摩擦支配，不是 ±1 N）—— 擦拭能用，精细抓取不行。
   加梁式称重传感器是最便宜的解法。
2. **`get_state()` / `get_positions()` 每周期都向总线发显式反馈请求帧**，高频调用可能压满 CAN。
   改用 `get_positions(request_feedback=False)` 走缓存。**最可能翻车的地方，M-1 就要测。**
3. **`mechVel` 不能用** —— 官方文档实测过：不是 rad/s，差 2.6~4.8 倍且符号不一致。
   **速度一律用位置差分。**
4. **⚠️ 测试时必须保持间隙** —— 官方标定文档里踩过这个坑：
   在静止姿势下（手爪碰到桌面）做扫掠，得到了"看起来 rms 很好"的**假结果**
   （假的 −0.40 rad 零偏、假的 ×1.66 手腕缩放）。
   **推荐姿势：肘部朝上的 "L" 形，j2≈0.7, j3≈1.1。**
5. **操作者是"盲画"** —— 力闭环解决了力的盲，但位置仍然看不见。建议在工作台上方放个
   摄像头，把画面打到界面上。
6. **线缆拖拽**会污染力估计，尤其是手臂伸远的时候。
7. **符号约定**：motor/mechPos 坐标系 ↔ 本仓库 URDF 是恒等映射；
   到 Seeed reBot-Isaacsim 的 URDF 是 `q_urdf = −q_motor`。跨仓库对数据时别搞错。
