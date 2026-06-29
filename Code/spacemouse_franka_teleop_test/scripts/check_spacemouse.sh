#!/usr/bin/env bash
set -eo pipefail

PKG_DIR="$(cd "$(dirname "$0")/.." && pwd)"
export PYTHONPATH="${PKG_DIR}:${PYTHONPATH:-}"

exec /home/admin123/WenshuoZhou/SERL/.venv/bin/python3 -m spacemouse_franka_teleop_test.check_spacemouse
