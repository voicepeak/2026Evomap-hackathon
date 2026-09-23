#!/bin/zsh
# 把 site/ 发布到 gh-pages 分支（不想用 GitHub Actions 时的后备方案）。
# 用法：双击本脚本，或在 Terminal 运行；然后在仓库
#   Settings → Pages → Source 选 “Deploy from a branch” → 分支 gh-pages / (root)
set -eu
ROOT="$(git -C "$(dirname "$0")" rev-parse --show-toplevel)"
SITE="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"

if [ -n "$(git status --porcelain)" ]; then
  echo "⚠️  工作区有未提交的改动，先提交再发布："
  git status --short | head -10
  exit 1
fi

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
echo "→ 导出 site/ 到临时目录"
git archive HEAD site | tar -x -C "$TMP"
cd "$TMP/site"

git init -q
git add -A
git -c user.name="$(git -C "$ROOT" config user.name)" \
    -c user.email="$(git -C "$ROOT" config user.email)" \
    commit -qm "publish site $(date '+%Y-%m-%d %H:%M')"
echo "→ 推送到 origin/gh-pages"
git push -f "$ROOT" HEAD:refs/heads/gh-pages

echo ""
echo "✅ 已发布到 gh-pages 分支。若还没开 Pages："
echo "   仓库 Settings → Pages → Source: Deploy from a branch → gh-pages / (root)"
echo "   站点地址: https://<用户名>.github.io/<仓库名>/"
