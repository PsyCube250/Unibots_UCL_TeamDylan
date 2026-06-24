#!/usr/bin/env bash
set -euo pipefail

# Removes the temporary pf anchor installed by mac_share_internet_to_jetson.sh.

ANCHOR="${ANCHOR:-com.apple/unibots}"

if [[ "$(id -u)" != "0" ]]; then
  echo "Please run with sudo:"
  echo "  sudo $0"
  exit 2
fi

pfctl -a "$ANCHOR" -F all
echo "Removed pf anchor: $ANCHOR"
echo "Note: IPv4 forwarding and pf may still be enabled if macOS or another service uses them."
