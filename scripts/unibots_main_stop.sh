#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
STM32_PORT="${UNIBOTS_STM32_PORT:-/dev/ttyTHS1}"
PYTHON_BIN="${UNIBOTS_PYTHON:-$REPO_DIR/unibots_ws/venv/bin/python}"

if [ ! -x "$PYTHON_BIN" ]; then
  PYTHON_BIN="python3"
fi

cd "$REPO_DIR"

"$PYTHON_BIN" robot_control/stm32_encoder_demo.py \
  --port "$STM32_PORT" \
  --send-stop \
  --send-stop-on-exit \
  --duration "${UNIBOTS_STOP_DURATION:-0.1}" >/dev/null 2>&1 || true
