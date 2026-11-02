#!/bin/bash
# Windows 側のソースを Linux 側ワークスペース (~/diffbot_ws) へコピーしてビルドする
# (/mnt/c 上で直接ビルドすると遅いため)
set -e
HERE="$(cd "$(dirname "$0")/.." && pwd)"
WS=~/diffbot_ws
mkdir -p "$WS/src"
rm -rf "$WS/src/diffbot_task"
cp -r "$HERE/ros2_ws/src/diffbot_task" "$WS/src/"
find "$WS/src/diffbot_task" -name __pycache__ -prune -exec rm -rf {} +
source /opt/ros/humble/setup.bash
cd "$WS" && colcon build --symlink-install
