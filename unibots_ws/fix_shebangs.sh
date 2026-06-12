#!/bin/bash
VENV_PYTHON="/home/jetson/Documents/Unibots/Unibots_UCL_TeamDylan/unibots_ws/venv/bin/python3"
WS="/home/jetson/Documents/Unibots/Unibots_UCL_TeamDylan/unibots_ws"

VENV_NODES=(
    "install/ball_detector/lib/ball_detector/ball_detector"
    "install/april_tag_detector/lib/april_tag_detector/april_tag_detector"
)

for node in "${VENV_NODES[@]}"; do
    if [ -f "$WS/$node" ]; then
        sed -i "s|#!/usr/bin/python3|#!$VENV_PYTHON|g" "$WS/$node"
        echo "Fixed shebang: $node"
    fi
done
