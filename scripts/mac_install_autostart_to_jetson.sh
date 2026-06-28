#!/usr/bin/env bash
set -euo pipefail

JETSON_SSH="${1:-${UNIBOTS_JETSON_SSH:-jetson@192.168.55.1}}"

quote_remote() {
  printf "'%s'" "$(printf "%s" "$1" | sed "s/'/'\\\\''/g")"
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "Jetson: $JETSON_SSH"
echo "Local repo: $REPO_DIR"
echo
echo "This will install the boot service in debug mode:"
echo "  UNIBOTS_AUTOSTART_ENABLED=0"
echo "The robot main code will NOT start during installation."
echo

REMOTE_REPO="$(
  ssh "$JETSON_SSH" 'set -e
for d in \
  /home/jetson/Documents/Unibots/Unibots_UCL_TeamDylan \
  /home/jetson/Unibots_UCL_TeamDylan \
  /home/jetson/Desktop/year2/Uni/Unibots_UCL_TeamDylan
do
  if [ -d "$d" ]; then
    echo "$d"
    exit 0
  fi
done
find /home/jetson -maxdepth 6 -type d -name Unibots_UCL_TeamDylan 2>/dev/null | head -n 1'
)"

if [ -z "$REMOTE_REPO" ]; then
  echo "Could not find Unibots_UCL_TeamDylan on the Jetson." >&2
  exit 1
fi

REMOTE_REPO_Q="$(quote_remote "$REMOTE_REPO")"
echo "Remote repo: $REMOTE_REPO"
echo

cd "$REPO_DIR"

tar -cf - \
  README.md \
  scripts/jetson_ros_yolo_env.sh \
  scripts/install_unibots_autostart.sh \
  scripts/unibots_autostart_switch.sh \
  scripts/unibots_main_boot.sh \
  scripts/unibots_main_stop.sh \
  systemd/system/unibots-main.service.in \
  | ssh "$JETSON_SSH" "cd $REMOTE_REPO_Q && tar -xf -"

ssh -tt "$JETSON_SSH" "cd $REMOTE_REPO_Q && \
  chmod +x scripts/install_unibots_autostart.sh scripts/unibots_autostart_switch.sh scripts/unibots_main_boot.sh scripts/unibots_main_stop.sh && \
  UNIBOTS_START_NOW=0 bash scripts/install_unibots_autostart.sh && \
  bash scripts/unibots_autostart_switch.sh off && \
  echo && \
  bash scripts/unibots_autostart_switch.sh status && \
  echo && \
  systemctl status unibots-main.service --no-pager -l || true"

echo
echo "Installed in debug mode. To allow main to run at next boot:"
echo "  bash $REMOTE_REPO/scripts/unibots_autostart_switch.sh on"
