# rebot-capture API（v0.1）

Base URL：`http://127.0.0.1:8787`

## REST

| 方法 | 路径 | 说明 | 请求体 / 参数 |
|---|---|---|---|
| GET | `/api/health` | 健康检查 | — |
| GET | `/api/profile` | 设备档案（关节/限位/增益/自动配置） | — |
| GET | `/api/device/status` | 设备状态（电机数/电压/温度/标定/故障） | — |
| GET | `/api/device/diagnostics` | 真机诊断（后端 + 映射器：q_cmd/v_cmd/qd/限位/夹爪） | — |
| POST | `/api/device/goto` | 把机械臂开到指定姿态（7 维，warmup，不计入数据） | `{target:[7], seconds?:1.2}` |
| POST | `/api/device/park` | 回折叠零位（最小 jerk，保持悬停） | `{}` |
| GET | `/api/teleop/state` | 控制核心状态（模式/冻结/速度档/直控/夹爪/预设/告警） | — |
| POST | `/api/teleop/freeze` | 冻结/解冻 | `{on?: bool}`（省略=切换） |
| POST | `/api/teleop/speed` | 速度档 慢/中/快 | `{index?: 0|1|2}`（省略=循环） |
| POST | `/api/teleop/mode` | 位置/姿态模式 | `{mode: "pos"|"ori"}` |
| POST | `/api/teleop/float` | 悬停（漂浮） | `{on: bool}` |
| POST | `/api/teleop/joint` | 关节直控（按住）/选关节 | `{index?: 0-6, step?: ±1, hold?: rad/s}`（`step`=服务端算上/下一个，不切模式） |
| POST | `/api/teleop/motor` | 选中电机并进入笔控直控（笔侧键单击循环 / `G` 直接选夹爪） | `{index?: 0-6, step?: ±1}`（省略 index 且给 step = 从**服务端当前**选择循环；6=夹爪） |
| POST | `/api/teleop/twist` | J6 自转速度 | `{value: rad/s}` |
| POST | `/api/teleop/align` | 重新对齐 | `{}` |
| POST | `/api/teleop/preset` | 预设 记录/前往 | `{action: "record"|"goto", index: 1-4}` |
| POST | `/api/teleop/tau_limit` | 力矩保护阈值（0=关） | `{value: N·m}` |
| GET | `/api/live` | 实时指标（fps/笔采样率/关节/质量灯） | — |
| GET | `/api/quality/rules` | 质量门阈值配置 | — |
| POST | `/api/session/start` | 开始会话 | `{task_id?, operator?}` |
| POST | `/api/session/stop` | 结束会话 | `{}` |
| POST | `/api/episode/start` | 开始录制当前 episode | `{}` |
| POST | `/api/episode/stop` | 结束并质检 | `{success: bool, note?}` |
| POST | `/api/episode/discard` | 丢弃当前 episode | `{}` |
| GET | `/api/episodes` | episode 列表 | `?details=true|false` |
| POST | `/api/pack` | 打包数据集（默认只含非 F 级） | `{name?, task_instruction?, include_failed?}` |
| GET | `/api/camera/status` | 相机列表 / 预览状态 | — |
| POST | `/api/camera/probe` | 探测相机 | `{}` |
| POST | `/api/camera/open` | 打开相机（本机索引或 MJPEG URL） | `{index?: 0-8, url?}` |
| POST | `/api/camera/close` | 关闭相机（省略=当前选中） | `{index?}` |
| POST | `/api/camera/alias` | 给相机起数据集里的名字（wrist/scene…） | `{name, index?, url?}` |
| GET | `/api/camera/stream` | MJPEG 预览流 | `?index=N`（省略=当前选中） |
| GET | `/api/gesture` | 手势夹爪状态（enabled/gesture/target/camera/error） | — |
| POST | `/api/gesture` | 启停手势夹爪（张开手=张开行程，握拳=闭合行程） | `{on?: bool}` |

错误约定：状态冲突（重复开始/未开始就结束）返回 `409 + detail`。

## WebSocket

| 路径 | 方向 | 载荷 |
|---|---|---|
| `/ws/teleop` | 客户端 → 服务端 | `{"type":"pen","data":{t,x,y,pressure,tiltX,tiltY,twist,touching}}`（也接受裸样本） |
| `/ws/state` | 服务端 → 客户端 | 10 Hz：`{"live": {...}, "status": {...}}` |

`x/y` 推荐归一化 0–1（浏览器）或毫米（真实数位板，由 Mapper 解释）；`pressure` 0–1。

## 返回示例

`POST /api/episode/stop`

```json
{
  "index": 3,
  "task_id": "grasp-and-place",
  "duration_s": 2.21,
  "frames": 221,
  "success": true,
  "grade": "A",
  "score": 93.7,
  "qc": [
    {"name": "drop_frame", "passed": true, "detail": "丢帧率 0.02%（上限 2%）", "value": 0.0002},
    {"name": "limit_occupancy", "passed": true, "detail": "限位占用 0.00%（上限 5%）", "value": 0.0}
  ],
  "score_detail": {
    "total": 93.7,
    "grade": "A",
    "hard_gate_passed": true,
    "breakdown": {"smoothness": 86.1, "completeness": 100.0, "sync": 90.5, "safety": 98.7, "privacy": 100.0},
    "metrics": {"duration_s": 2.21, "frames": 221, "drop_rate": 0.0002, "limit_occupancy": 0.0, "max_gap_ms": 11.2, "mean_abs_jerk": 74.8}
  }
}
```

`POST /api/pack`

```json
{
  "dataset": "rebot_20260923_120000",
  "path": ".../data/datasets/rebot_20260923_120000",
  "episodes": 2,
  "frames": 440,
  "grades": {"A": 2},
  "size_mb": 0.42,
  "parquet": true
}
```
