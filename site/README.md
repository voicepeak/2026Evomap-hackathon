# site/ —— 项目站点（GitHub Pages）

零依赖静态页（原生 JS / 无 CDN），**388KB**，离线双击 `index.html` 也能看。

## 本地预览

```bash
cd site && ../capture_platform/.venv/bin/python -m http.server 8899
# 打开 http://127.0.0.1:8899/
```

## 发布

两种方式（任选，仓库 Settings → Pages 里选对应的 Source）：

| 方式 | 设置 | 说明 |
|---|---|---|
| GitHub Actions（推荐） | Source = **GitHub Actions** | 推 `site/**` 即自动发布，workflow 见 `.github/workflows/pages.yml` |
| gh-pages 分支 | Source = **Deploy from a branch** → `gh-pages` / (root) | 跑 `./site/publish.command` 手动发布 |

站点地址：`https://voicepeak.github.io/2026Evomap-hackathon/`

## 五类交互（`app.js`）

1. 悬停：卡片指针聚光 + 3D 倾斜 + 按钮磁吸
2. 滚动 morph：SVG 形状在「笔 → 关节轨迹 → 数据集」三态间连续变形
3. 拖拽物理：Canvas 把 episode 拖进 `datasets/`（A/B 入库、F 被弹回）
4. 滚动 3D 穿越：输入→采集→平台→商业 四层
5. 指针视差：hero 分层 + 背景粒子

所有动画都尊重 `prefers-reduced-motion`（系统「减弱动态效果」时静态呈现）。

## 素材来源（都可核验）

| 文件 | 来源 |
|---|---|
| `assets/traj.jpg` | `make_traj.py` 从真实 parquet 直接画（`data/episodes/20260923_135010`，7 自由度含夹爪） |
| `assets/gui_panel.jpg` | `capture_platform/scripts/shot_window.py` 抓的 GUI 窗口图，已裁掉顶栏（**不含 mock 字样、不含桌面**） |
| `assets/wrist.webp` | `data/episodes/20260923_181011_ep000001.mp4` 抽 24 帧转 WebP |
| `docs/screenshots/*_clean.png` | 同上的原始干净截图（2857×1787 / 2008×1170），作品图片可直接用 |

重新生成：

```bash
# 轨迹图（改数据源改 SRC）
capture_platform/.venv/bin/python site/make_traj.py

# 界面截图（GUI 跑起来后，按窗口抓，不受遮挡/不拍桌面）
capture_platform/.venv/bin/python capture_platform/scripts/shot_window.py --list
capture_platform/.venv/bin/python capture_platform/scripts/shot_window.py --title "reBot" -o /tmp/gui.png
```

> 注意：作品图片/演示视频里**不要出现 mock 字样**。用 `--backend rebot` 起真机，
> 或直接裁掉顶栏（本目录的 jpg 已处理）。
