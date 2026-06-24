#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-probe}"
PORT="${UNIBOTS_STM32_PORT:-/dev/ttyTHS1}"
BAUD="${UNIBOTS_STM32_BOOT_BAUD:-115200}"
ROOT="${UNIBOTS_ROOT:-$HOME/Documents/Unibots/Unibots_UCL_TeamDylan}"
FIRMWARE="${UNIBOTS_STM32_FIRMWARE:-$ROOT/stm32/proposed_firmware/four_wheel_encoder_demo/firmware.bin}"
TOOL_DIR="$HOME/tools/stm32flash_pkg"
TOOL="$TOOL_DIR/usr/bin/stm32flash"

usage() {
  echo "Usage: $0 probe|write|go"
  echo
  echo "Before probe/write with the STM32 ROM bootloader:"
  echo "  1. Keep motor power off if possible; otherwise keep wheels lifted and power supervised."
  echo "  2. Set STM32 BOOT0 to 3.3V."
  echo "  3. Reset/power-cycle the STM32."
  echo "  4. After flashing, set BOOT0 back to GND and reset again."
}

ensure_tool() {
  if [ -x "$TOOL" ]; then
    return
  fi

  mkdir -p "$TOOL_DIR"
  cd "$TOOL_DIR"
  apt-get download stm32flash
  dpkg-deb -x stm32flash_*.deb .
}

ensure_tool

case "$MODE" in
  probe)
    echo "Probing STM32 bootloader on $PORT at $BAUD 8E1; no flash write."
    timeout 6 "$TOOL" -b "$BAUD" "$PORT"
    ;;
  write)
    if [ ! -s "$FIRMWARE" ]; then
      echo "Firmware not found: $FIRMWARE" >&2
      exit 1
    fi
    echo "Writing $FIRMWARE to STM32 on $PORT at $BAUD 8E1."
    "$TOOL" -b "$BAUD" -w "$FIRMWARE" -v -g 0x0 "$PORT"
    ;;
  go)
    echo "Starting flash application on $PORT."
    "$TOOL" -b "$BAUD" -g 0x0 "$PORT"
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
