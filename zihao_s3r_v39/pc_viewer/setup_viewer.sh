#!/bin/sh
# 作者：同济子豪兄 2026-10-26 | RoBoBo V3.9
set -eu
cd "$(dirname "$0")"
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
