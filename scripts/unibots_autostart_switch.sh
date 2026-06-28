#!/usr/bin/env bash
set -euo pipefail

CONFIG_FILE="${UNIBOTS_CONFIG_FILE:-/etc/default/unibots-main}"

usage() {
  cat <<EOF
Usage:
  scripts/unibots_autostart_switch.sh status
  scripts/unibots_autostart_switch.sh on
  scripts/unibots_autostart_switch.sh off

Controls the final boot switch:
  $CONFIG_FILE
  UNIBOTS_AUTOSTART_ENABLED=0 or 1
EOF
}

ensure_config() {
  if [ -f "$CONFIG_FILE" ]; then
    return
  fi
  tmp="$(mktemp)"
  cat > "$tmp" <<'EOF'
# Final boot autostart switch for the Unibots main robot service.
# 0 = normal Jetson boot, do not run main automatically.
# 1 = run main automatically when the Jetson boots.
UNIBOTS_AUTOSTART_ENABLED=0
EOF
  sudo install -m 0644 "$tmp" "$CONFIG_FILE"
  rm -f "$tmp"
}

set_switch() {
  local value="$1"
  ensure_config
  tmp="$(mktemp)"
  if grep -q '^UNIBOTS_AUTOSTART_ENABLED=' "$CONFIG_FILE"; then
    sed "s/^UNIBOTS_AUTOSTART_ENABLED=.*/UNIBOTS_AUTOSTART_ENABLED=$value/" "$CONFIG_FILE" > "$tmp"
  else
    cat "$CONFIG_FILE" > "$tmp"
    printf '\nUNIBOTS_AUTOSTART_ENABLED=%s\n' "$value" >> "$tmp"
  fi
  sudo install -m 0644 "$tmp" "$CONFIG_FILE"
  rm -f "$tmp"
}

status() {
  if [ ! -f "$CONFIG_FILE" ]; then
    echo "UNIBOTS_AUTOSTART_ENABLED=0 (config missing; default off)"
    return
  fi
  value="$(awk -F= '$1 == "UNIBOTS_AUTOSTART_ENABLED" {print $2; found=1} END {if (!found) print "0"}' "$CONFIG_FILE")"
  echo "UNIBOTS_AUTOSTART_ENABLED=$value"
  if command -v systemctl >/dev/null 2>&1; then
    systemctl is-enabled unibots-main.service 2>/dev/null | sed 's/^/service enabled: /' || true
    systemctl is-active unibots-main.service 2>/dev/null | sed 's/^/service active: /' || true
  fi
}

case "${1:-}" in
  on|enable|enabled|1|true)
    set_switch 1
    sudo systemctl enable unibots-main.service >/dev/null 2>&1 || true
    echo "UNIBOTS_AUTOSTART_ENABLED=1"
    ;;
  off|disable|disabled|0|false)
    set_switch 0
    sudo systemctl stop unibots-main.service >/dev/null 2>&1 || true
    echo "UNIBOTS_AUTOSTART_ENABLED=0"
    ;;
  status|"")
    status
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
