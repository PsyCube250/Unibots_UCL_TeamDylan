#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  bash scripts/mac_set_home_tags_on_jetson.sh TAG_A TAG_B
  bash scripts/mac_set_home_tags_on_jetson.sh TAG_A,TAG_B
  bash scripts/mac_set_home_tags_on_jetson.sh --restart TAG_A TAG_B

Examples:
  bash scripts/mac_set_home_tags_on_jetson.sh 20 21
  bash scripts/mac_set_home_tags_on_jetson.sh 5,12

Environment:
  UNIBOTS_JETSON_SSH=jetson@192.168.0.237
  UNIBOTS_JETSON_KEY=$HOME/.ssh/id_ed25519_unibots

Default behavior only updates /etc/default/unibots-main for the next boot.
--restart restarts the live service immediately, so the robot may move.
EOF
}

quote_remote() {
  printf "'%s'" "$(printf "%s" "$1" | sed "s/'/'\\\\''/g")"
}

validate_tag() {
  local tag="$1"
  if ! [[ "$tag" =~ ^[0-9]+$ ]]; then
    echo "Invalid tag ID: $tag" >&2
    exit 2
  fi
  if [ "$tag" -gt 586 ]; then
    echo "Tag ID $tag is outside AprilTag 36h11 range 0..586." >&2
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

if [ "$#" -eq 1 ] && [[ "$1" == *,* ]]; then
  IFS=',' read -r TAG_A TAG_B EXTRA <<<"$1"
  if [ -n "${EXTRA:-}" ]; then
    echo "Expected exactly two tag IDs, got: $1" >&2
    exit 2
  fi
elif [ "$#" -eq 2 ]; then
  TAG_A="$1"
  TAG_B="$2"
else
  usage >&2
  exit 2
fi

validate_tag "$TAG_A"
validate_tag "$TAG_B"
NEW_IDS="${TAG_A},${TAG_B}"

SSH_OPTS=(
  -o ConnectTimeout=8
  -o UserKnownHostsFile=/tmp/unibots_jetson_known_hosts
  -o StrictHostKeyChecking=accept-new
)
if [ -f "$JETSON_KEY" ]; then
  SSH_OPTS+=(-i "$JETSON_KEY")
fi

echo "Jetson: $JETSON_SSH"
echo "Home AprilTag IDs: $NEW_IDS"
if [ "$RESTART" = "1" ]; then
  echo "Mode: update config and restart live service"
else
  echo "Mode: update config only; use the next power cycle/restart"
fi
echo

NEW_IDS_Q="$(quote_remote "$NEW_IDS")"
RESTART_Q="$(quote_remote "$RESTART")"

ssh -tt "${SSH_OPTS[@]}" "$JETSON_SSH" "NEW_IDS=$NEW_IDS_Q RESTART=$RESTART_Q bash -s" <<'REMOTE'
set -euo pipefail

CONFIG_FILE="/etc/default/unibots-main"

if [ ! -f "$CONFIG_FILE" ]; then
  echo "Missing $CONFIG_FILE on Jetson." >&2
  exit 1
fi

echo "Updating $CONFIG_FILE ..."
sudo cp "$CONFIG_FILE" "${CONFIG_FILE}.bak.$(date +%Y%m%d_%H%M%S)"

if sudo grep -q '^UNIBOTS_HOME_TAG_IDS=' "$CONFIG_FILE"; then
  sudo sed -i "s/^UNIBOTS_HOME_TAG_IDS=.*/UNIBOTS_HOME_TAG_IDS=$NEW_IDS/" "$CONFIG_FILE"
else
  printf '\nUNIBOTS_HOME_TAG_IDS=%s\n' "$NEW_IDS" | sudo tee -a "$CONFIG_FILE" >/dev/null
fi

echo
grep '^UNIBOTS_HOME_TAG_IDS=' "$CONFIG_FILE"
echo
systemctl is-enabled unibots-main.service | sed 's/^/service enabled: /' || true
systemctl is-active unibots-main.service | sed 's/^/service active: /' || true

if [ "$RESTART" = "1" ]; then
  echo
  echo "Restarting unibots-main.service. The robot may move."
  sudo systemctl restart unibots-main.service
  sleep 2
  systemctl is-active unibots-main.service | sed 's/^/service active after restart: /' || true
else
  echo
  echo "Done. Power-cycle or restart the service before the match to use these IDs."
fi
REMOTE
