#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  bash scripts/mac_set_motor_power_on_jetson.sh safe
  bash scripts/mac_set_motor_power_on_jetson.sh boost
  bash scripts/mac_set_motor_power_on_jetson.sh LIMIT APPROACH CREEP COLLECT TURN SEARCH
  bash scripts/mac_set_motor_power_on_jetson.sh --restart boost

Presets:
  safe   = 45 39 36 30 34 39
  boost  = 80 60 50 45 60 60

Default behavior updates /etc/default/unibots-main for the next power cycle.
--restart restarts the live service immediately, so the robot may move.
EOF
}

quote_remote() {
  printf "'%s'" "$(printf "%s" "$1" | sed "s/'/'\\\\''/g")"
}

validate_pwm() {
  local name="$1"
  local value="$2"
  if ! [[ "$value" =~ ^[0-9]+$ ]]; then
    echo "$name must be an integer: $value" >&2
    exit 2
  fi
  if [ "$value" -lt 0 ] || [ "$value" -gt 80 ]; then
    echo "$name must be 0..80 for the current STM32 firmware safety cap: $value" >&2
    exit 2
  fi
}

RESTART=0
JETSON_SSH="${UNIBOTS_JETSON_SSH:-jetson@192.168.0.237}"
JETSON_KEY="${UNIBOTS_JETSON_KEY:-$HOME/.ssh/id_ed25519_unibots}"

while [ "$#" -gt 0 ]; do
  case "$1" in
    --restart)
      RESTART=1
      shift
      ;;
    --ssh)
      JETSON_SSH="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      break
      ;;
  esac
done

case "${1:-}" in
  safe)
    LIMIT=45
    APPROACH=39
    CREEP=36
    COLLECT=30
    TURN=34
    SEARCH=39
    ;;
  boost)
    LIMIT=80
    APPROACH=60
    CREEP=50
    COLLECT=45
    TURN=60
    SEARCH=60
    ;;
  "")
    usage >&2
    exit 2
    ;;
  *)
    if [ "$#" -ne 6 ]; then
      usage >&2
      exit 2
    fi
    LIMIT="$1"
    APPROACH="$2"
    CREEP="$3"
    COLLECT="$4"
    TURN="$5"
    SEARCH="$6"
    ;;
esac

validate_pwm LIMIT "$LIMIT"
validate_pwm APPROACH "$APPROACH"
validate_pwm CREEP "$CREEP"
validate_pwm COLLECT "$COLLECT"
validate_pwm TURN "$TURN"
validate_pwm SEARCH "$SEARCH"

for value in "$APPROACH" "$CREEP" "$COLLECT" "$TURN" "$SEARCH"; do
  if [ "$value" -gt "$LIMIT" ]; then
    echo "All PWM values must be <= LIMIT. Got value=$value limit=$LIMIT" >&2
    exit 2
  fi
done

SSH_OPTS=(
  -o ConnectTimeout=8
  -o UserKnownHostsFile=/tmp/unibots_jetson_known_hosts
  -o StrictHostKeyChecking=accept-new
)
if [ -f "$JETSON_KEY" ]; then
  SSH_OPTS+=(-i "$JETSON_KEY")
fi

echo "Jetson: $JETSON_SSH"
echo "Motor power: limit=$LIMIT approach=$APPROACH creep=$CREEP collect=$COLLECT turn=$TURN search=$SEARCH"
if [ "$RESTART" = "1" ]; then
  echo "Mode: update config and restart live service"
else
  echo "Mode: update config only; use next power cycle/restart"
fi
echo

ssh -tt "${SSH_OPTS[@]}" "$JETSON_SSH" \
  "LIMIT=$(quote_remote "$LIMIT") APPROACH=$(quote_remote "$APPROACH") CREEP=$(quote_remote "$CREEP") COLLECT=$(quote_remote "$COLLECT") TURN=$(quote_remote "$TURN") SEARCH=$(quote_remote "$SEARCH") RESTART=$(quote_remote "$RESTART") bash -s" <<'REMOTE'
set -euo pipefail

CONFIG_FILE="/etc/default/unibots-main"

if [ ! -f "$CONFIG_FILE" ]; then
  echo "Missing $CONFIG_FILE on Jetson." >&2
  exit 1
fi

sudo cp "$CONFIG_FILE" "${CONFIG_FILE}.bak.$(date +%Y%m%d_%H%M%S)"

set_key() {
  local key="$1"
  local value="$2"
  if sudo grep -q "^${key}=" "$CONFIG_FILE"; then
    sudo sed -i "s/^${key}=.*/${key}=${value}/" "$CONFIG_FILE"
  else
    printf '%s=%s\n' "$key" "$value" | sudo tee -a "$CONFIG_FILE" >/dev/null
  fi
}

set_key UNIBOTS_PWM_LIMIT "$LIMIT"
set_key UNIBOTS_APPROACH_PWM "$APPROACH"
set_key UNIBOTS_CREEP_PWM "$CREEP"
set_key UNIBOTS_COLLECT_DRIVE_PWM "$COLLECT"
set_key UNIBOTS_TURN_PWM "$TURN"
set_key UNIBOTS_SEARCH_PWM "$SEARCH"

grep -E 'UNIBOTS_(PWM_LIMIT|APPROACH|CREEP|COLLECT_DRIVE|TURN|SEARCH)_PWM=' "$CONFIG_FILE"

if [ "$RESTART" = "1" ]; then
  echo
  echo "Restarting unibots-main.service. The robot may move."
  sudo systemctl restart unibots-main.service
  sleep 2
  systemctl is-active unibots-main.service | sed 's/^/service active after restart: /' || true
else
  echo
  echo "Done. Power-cycle or restart the service to use these motor settings."
fi
REMOTE
