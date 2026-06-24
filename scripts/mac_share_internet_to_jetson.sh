#!/usr/bin/env bash
set -euo pipefail

# Temporary Mac NAT for Jetson USB device-mode networking.
# Mac Wi-Fi/LAN side: en0
# Jetson USB side:   en9, Mac address 192.168.55.100, Jetson 192.168.55.1
#
# Run on the Mac:
#   sudo scripts/mac_share_internet_to_jetson.sh

WAN_IF="${WAN_IF:-en0}"
JETSON_IF="${JETSON_IF:-en9}"
JETSON_NET="${JETSON_NET:-192.168.55.0/24}"
ANCHOR="${ANCHOR:-com.apple/unibots}"
RULE_FILE="/tmp/unibots-jetson-nat.pf"

if [[ "$(id -u)" != "0" ]]; then
  echo "Please run with sudo:"
  echo "  sudo $0"
  exit 2
fi

if ! ifconfig "$WAN_IF" >/dev/null 2>&1; then
  echo "Missing WAN interface: $WAN_IF"
  exit 1
fi

if ! ifconfig "$JETSON_IF" >/dev/null 2>&1; then
  echo "Missing Jetson USB interface: $JETSON_IF"
  echo "Reconnect Jetson USB-C device-mode cable and retry."
  exit 1
fi

cat > "$RULE_FILE" <<EOF
nat on $WAN_IF inet from $JETSON_NET to any -> ($WAN_IF)
pass quick on $JETSON_IF inet from $JETSON_NET to any keep state
pass quick on $WAN_IF inet proto { tcp udp icmp } from ($WAN_IF) to any keep state
EOF

echo "Enabling IPv4 forwarding..."
sysctl -w net.inet.ip.forwarding=1

echo "Loading pf NAT anchor $ANCHOR from $RULE_FILE..."
pfctl -a "$ANCHOR" -f "$RULE_FILE"
pfctl -E >/dev/null 2>&1 || true

echo "Mac -> Jetson temporary NAT is enabled."
echo "WAN_IF=$WAN_IF JETSON_IF=$JETSON_IF JETSON_NET=$JETSON_NET"
echo "To remove it later:"
echo "  sudo scripts/mac_unshare_internet_to_jetson.sh"
