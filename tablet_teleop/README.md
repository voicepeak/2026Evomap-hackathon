# tablet_teleop · M0

数位板采集 + 可视化。**纯前端，不依赖机械臂**，是整套方案的第一步，也是最后的降级方案。

```
tablet_teleop/
├── web/index.html   # 全部前端逻辑（单文件，无依赖）
├── bridge.py        # 可选：WebSocket 服务，把样本送进 Python + 写 CSV
└── README.md
```

---

## 跑起来

### 1. 只看看（不需要 Python）

```bash
open web/index.html
```

用笔在板子上划。**就算不连 bridge 也能用**，所有可视化都在本地。

### 2. 接上 Python（M1 的入口）

```bash
pip install websockets
python bridge.py                 # 只打印
python bridge.py --csv out.csv   # 顺便记录
```

然后在网页右上角点 **「连接 bridge」**。

> 如果浏览器从 `file://` 连不上 WebSocket，起个本地服务：
> `cd web && python3 -m http.server 8000`，然后开 `http://localhost:8000`

---

## 打开后先看这几个数字

| 看哪里 | 应该是什么 | 不对的话说明 |
|---|---|---|
| **pointerType** | `pen` | 显示 `mouse` → Wacom 驱动把笔当鼠标了，去驱动设置里关掉鼠标模式 |
| **采样率** | 约 **200 Hz** | 只有 60 左右 → `getCoalescedEvents()` 没生效，或换浏览器试试 |
| **合并** | **> 1**（通常 2~4） | 一直是 1 → 数位板只按帧率上报，不是 200Hz 设备 |
| **pressure** | 用力按能到 **1.000**，松开接近 0 | 完全不到 1 → 该型号压力级数低（入门款常见） |
| **alt / az** | 有弧度值 | 显示 `–` → **这块板子不上报倾斜**，映射表要删掉倾角那两行 |
| **twist** | 转动笔杆会变 | 一直是 0 → 同上，没有旋转通道 |
| **可达半径** | 光标落在绿圆内 | 红色 → 映射超出 70% 臂展，机械臂会堵转掉臂 |

**这几行就是 M0 的全部目的**：在碰机械臂之前，把「数位板到底能给我什么」这件事搞清楚。

---

## 界面上的东西

- **板面画布**：线条粗细/亮度 = 笔压，就是"压得重更亮"那个映射的预览
- **法向力指令**：`F = F_min + 笔压 × (F_max − F_min)`，这是将来喂给机械臂的核心通道
- **波形**：蓝色是笔压，橙色是力指令，5 秒滚动
- **笔姿态**：侧视图，笔杆与板面夹角 = altitudeAngle
- **工作空间预览**：俯视。橙色点是机械臂基座，圆圈是可达范围，
  蓝框是板面映射过去的工作台，绿/红点是你笔尖现在对应的位置
- **映射参数**：板面尺寸、增益、工作台距基座的距离、力的上下限、可达半径
- **记录 / 导出 CSV**：含全部通道 + 力指令，可直接喂给后面的数据管线

---

## 已知限制

- 鼠标也能画，但 `pointerType` 会是 `mouse`，压力恒为 0.5（按下时）。**必须用笔。**
- 触控被忽略（`pointerType === 'touch'` 直接 return），避免手掌误触。
- `pointerrawupdate` 没用（Chrome 专属），走的是 `pointermove` + `getCoalescedEvents()`，
  兼容性更好，200Hz 设备足够。
- 工作空间预览里，可达半径是按基座同心圆画的近似值，没考虑关节限位和自碰撞。

---

## 下一步（M1）

1. 在 `bridge.py` 的 `ArmSink.on_sample()` 里接上 `reBotArm_control_py`
2. 把 `BUF.latest()` 的 `(x, y, F)` 变成 `TaskCmd`
3. 实现 `frame_jacobian()`（Pinocchio）
4. **先测控制率**（见 `../tablet_teleop_PLAN.md` §0.5 的 M-1）
