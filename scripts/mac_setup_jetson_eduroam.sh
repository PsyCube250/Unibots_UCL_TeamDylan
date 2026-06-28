#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  bash scripts/mac_setup_jetson_eduroam.sh
  bash scripts/mac_setup_jetson_eduroam.sh --ssh jetson@192.168.55.1

This connects to the Jetson over SSH and prompts for the eduroam
username/password inside the SSH terminal. NetworkManager stores the
profile on the Jetson, so it should auto-connect on future boots.

Environment:
  UNIBOTS_JETSON_SSH=jetson@192.168.55.1
  UNIBOTS_JETSON_KEY=$HOME/.ssh/id_ed25519_unibots
EOF
}

JETSON_SSH="${UNIBOTS_JETSON_SSH:-jetson@192.168.55.1}"
JETSON_KEY="${UNIBOTS_JETSON_KEY:-$HOME/.ssh/id_ed25519_unibots}"

while [ "$#" -gt 0 ]; do
  case "$1" in
    --ssh)
      JETSON_SSH="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      usage >&2
      exit 2
      ;;
  esac
done

SSH_OPTS=(
  -o ConnectTimeout=8
  -o UserKnownHostsFile=/tmp/unibots_usb_known_hosts
  -o StrictHostKeyChecking=accept-new
)
if [ -f "$JETSON_KEY" ]; then
  SSH_OPTS+=(-i "$JETSON_KEY")
fi

echo "Jetson: $JETSON_SSH"
echo "This will configure eduroam on the Jetson and save it for autoconnect."
echo "Do not paste the eduroam password into chat; type it only at the terminal prompt."
echo

REMOTE_SCRIPT="/tmp/unibots_setup_eduroam_$$.sh"

ssh "${SSH_OPTS[@]}" "$JETSON_SSH" "cat > '$REMOTE_SCRIPT' && chmod 700 '$REMOTE_SCRIPT'" <<'REMOTE'
set -euo pipefail

CON_NAME="eduroam"
SSID="eduroam"

wifi_dev="$(nmcli -t -f DEVICE,TYPE,STATE device status | awk -F: '$2=="wifi"{print $1; exit}')"
if [ -z "$wifi_dev" ]; then
  echo "No Wi-Fi device found on this Jetson." >&2
  exit 1
fi

existing_identity="$(nmcli -g 802-1x.identity connection show "$CON_NAME" 2>/dev/null || true)"
default_identity="${existing_identity:-}"

echo "Wi-Fi device: $wifi_dev"
echo "eduroam signal check:"
nmcli -f SSID,SIGNAL,SECURITY dev wifi list --rescan yes | awk 'NR==1 || $1=="eduroam" {print}'
echo

if [ -n "$default_identity" ]; then
  read -r -p "eduroam identity [$default_identity]: " identity
  identity="${identity:-$default_identity}"
else
  read -r -p "eduroam identity, e.g. abc123@ucl.ac.uk: " identity
fi

if [ -z "$identity" ]; then
  echo "Identity cannot be empty." >&2
  exit 2
fi

read -r -s -p "eduroam password: " eduroam_password
echo
if [ -z "$eduroam_password" ]; then
  echo "Password cannot be empty." >&2
  exit 2
fi

echo
echo "Creating/updating NetworkManager eduroam profile..."

if ! sudo -n true 2>/dev/null; then
  echo "Jetson sudo password may be requested once."
fi

sudo nmcli radio wifi on

if ! sudo nmcli -t -f NAME connection show | grep -Fxq "$CON_NAME"; then
  sudo nmcli connection add \
    type wifi \
    ifname "$wifi_dev" \
    con-name "$CON_NAME" \
    ssid "$SSID"
fi

sudo nmcli connection modify "$CON_NAME" \
  connection.autoconnect yes \
  connection.autoconnect-priority 50 \
  802-11-wireless.ssid "$SSID" \
  wifi-sec.key-mgmt wpa-eap \
  802-1x.eap peap \
  802-1x.phase2-auth mschapv2 \
  802-1x.identity "$identity" \
  802-1x.anonymous-identity "anonymous@ucl.ac.uk" \
  802-1x.domain-suffix-match "ucl.ac.uk" \
  802-1x.system-ca-certs yes \
  802-1x.password-flags 0 \
  802-1x.password "$eduroam_password" \
  ipv4.method auto \
  ipv6.method auto

unset eduroam_password

echo
echo "Connecting..."
sudo nmcli connection up "$CON_NAME"

echo
echo "Status:"
nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device status
ip -4 addr show "$wifi_dev" | awk '/inet / {print "Wi-Fi IPv4: "$2}'
nmcli -g GENERAL.CONNECTION,IP4.ADDRESS device show "$wifi_dev" | sed 's/^/  /'
echo
echo "Done. The Jetson should now auto-connect to eduroam after reboot."
REMOTE

ssh -tt "${SSH_OPTS[@]}" "$JETSON_SSH" "bash '$REMOTE_SCRIPT'; rc=\$?; rm -f '$REMOTE_SCRIPT'; exit \$rc"
