#!/bin/zsh
# 环境自愈：把 capture_platform/.venv 的“基础 Python”指回工作区自带的副本。
#
# 背景：这个 venv 最早是用 Codex 运行时缓存里的 Python 3.12 建的
#       （~/.cache/codex-runtimes/.../python）。缓存目录可能被清理/更新，
#       一旦消失，venv 里所有依赖（含 PySide6 / pinocchio / mediapipe）都会失效。
#       工作区已保留一份运行时副本：.rollback/runtime_python_20260923/
#
# 用法：双击本脚本（或在 Terminal 运行），然后重新启动服务/GUI。
set -eu
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"        # 工作区根
RM="$ROOT/.rollback/runtime_python_20260923"
VENV="$ROOT/capture_platform/.venv"

if [ ! -x "$RM/bin/python3.12" ]; then
  echo "❌ 缺少运行时副本：$RM/bin/python3.12"
  echo "   可从 ~/.cache/codex-runtimes/codex-primary-runtime/dependencies/python 复制一份到该路径。"
  exit 1
fi

ln -sfn "$RM/bin/python3" "$VENV/bin/python3"
python3 - "$VENV/pyvenv.cfg" "$RM" <<'PY'
import sys
cfg, rm = sys.argv[1], sys.argv[2]
out = []
for line in open(cfg, encoding="utf-8"):
    if line.startswith("home ="):
        line = f"home = {rm}/bin\n"
    elif line.startswith("executable ="):
        line = f"executable = {rm}/bin/python3.12\n"
    out.append(line)
open(cfg, "w", encoding="utf-8").writelines(out)
PY

echo "环境已指向工作区运行时副本："
"$VENV/bin/python" -c "
import sys
print('  python  :', sys.version.split()[0])
print('  base    :', sys.base_prefix)
import numpy, pyarrow, cv2, fastapi
print('  依赖    : numpy/pyarrow/opencv/fastapi OK')
try:
    import PySide6; print('  GUI     : PySide6', PySide6.__version__, 'OK')
except Exception as e:
    print('  GUI     : 导入失败 ->', e)
try:
    import pinocchio; print('  真机    : pinocchio OK')
except Exception as e:
    print('  真机    : 导入失败 ->', e)
"
echo "完成。可继续运行 scripts/run_gui.command 或 scripts/run_serve.command"
