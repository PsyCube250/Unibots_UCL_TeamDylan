#!/usr/bin/env bash
set -euo pipefail

if ! command -v systemctl >/dev/null 2>&1; then
  echo "This installer must run on the Jetson/Linux system with systemd." >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
RUN_USER="${UNIBOTS_SERVICE_USER:-${SUDO_USER:-$USER}}"
CONFIG_FILE="${UNIBOTS_CONFIG_FILE:-/etc/default/unibots-main}"
UNIT_TMP="$(mktemp)"

sed \
  -e "s|@REPO_DIR@|$REPO_DIR|g" \
  -e "s|@RUN_USER@|$RUN_USER|g" \
  "$REPO_DIR/systemd/system/unibots-main.service.in" > "$UNIT_TMP"

sudo install -m 0644 "$UNIT_TMP" /etc/systemd/system/unibots-main.service
rm -f "$UNIT_TMP"

if [ ! -f "$CONFIG_FILE" ]; then
  CONFIG_TMP="$(mktemp)"
  cat > "$CONFIG_TMP" <<'EOF'
# Final boot autostart switch for the Unibots main robot service.
# 0 = normal Jetson boot, do not run main automatically.
# 1 = run main automatically when the Jetson boots.
UNIBOTS_AUTOSTART_ENABLED=0
EOF
  sudo install -m 0644 "$CONFIG_TMP" "$CONFIG_FILE"
  rm -f "$CONFIG_TMP"
fi

sudo systemctl daemon-reload
sudo systemctl enable unibots-main.service

if [ "${UNIBOTS_START_NOW:-0}" = "1" ]; then
  sudo systemctl restart unibots-main.service
fi

cat <<EOF
Installed /etc/systemd/system/unibots-main.service
Config: $CONFIG_FILE
Run user: $RUN_USER
Repo: $REPO_DIR

Final autostart switch:
  bash $REPO_DIR/scripts/unibots_autostart_switch.sh status
  bash $REPO_DIR/scripts/unibots_autostart_switch.sh on
  bash $REPO_DIR/scripts/unibots_autostart_switch.sh off

Status:
  systemctl status unibots-main.service

Logs:
  journalctl -u unibots-main.service -f
  tail -f $REPO_DIR/logs/unibots_main_boot.log

Stop and send STM32 STOP:
  sudo systemctl stop unibots-main.service
  bash $REPO_DIR/scripts/unibots_main_stop.sh

Disable autostart:
  bash $REPO_DIR/scripts/unibots_autostart_switch.sh off
  sudo systemctl disable --now unibots-main.service
EOF
