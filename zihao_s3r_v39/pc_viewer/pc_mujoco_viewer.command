#!/bin/sh
# 作者：同济子豪兄 2026-10-26 | RoBoBo V3.9
set -eu
cd "$(dirname "$0")"
if [ ! -x .venv/bin/mjpython ]; then
  echo "请先运行 setup_viewer.sh 安装查看器依赖。"
  exit 1
fi
exec .venv/bin/mjpython live_robot_viewer.py "$@"
