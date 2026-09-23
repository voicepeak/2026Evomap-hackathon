"""路径与目录约定。

所有产物默认落在项目内 `data/`（可用环境变量 REBOT_CAPTURE_HOME 覆盖）：

    data/
    ├── sessions/           # 原始会话（episode 文件）
    ├── datasets/           # 打包后的 LeRobot 数据集
    └── logs/               # 自检/服务日志
"""
from __future__ import annotations

import os
from pathlib import Path

PKG_DIR = Path(__file__).resolve().parent
REPO_DIR = PKG_DIR.parent
CONFIGS_DIR = REPO_DIR / "configs"
DEFAULT_PROFILE = CONFIGS_DIR / "rebot_b601_rs.json"


def data_home() -> Path:
    override = os.environ.get("REBOT_CAPTURE_HOME")
    root = Path(override).expanduser() if override else REPO_DIR / "data"
    root.mkdir(parents=True, exist_ok=True)
    return root


def sessions_dir() -> Path:
    p = data_home() / "sessions"
    p.mkdir(parents=True, exist_ok=True)
    return p


def datasets_dir() -> Path:
    p = data_home() / "datasets"
    p.mkdir(parents=True, exist_ok=True)
    return p


def episodes_dir() -> Path:
    """episode 原始素材（视频等）。"""
    p = data_home() / "episodes"
    p.mkdir(parents=True, exist_ok=True)
    return p


def logs_dir() -> Path:
    p = data_home() / "logs"
    p.mkdir(parents=True, exist_ok=True)
    return p


def web_dir() -> Path:
    return PKG_DIR / "web"
