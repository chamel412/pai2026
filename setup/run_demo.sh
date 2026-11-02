#!/bin/bash
# ビルド → 自律巡回デモ実行 → H.264 に変換
# 実行: bash run_demo.sh [出力ディレクトリ]
set -e
HERE="$(cd "$(dirname "$0")/.." && pwd)"
OUT_DIR="${1:-$HERE/output}"
WS=~/diffbot_ws
mkdir -p "$OUT_DIR" "$WS/src"

# Windows 側のソースを Linux 側ワークスペースへコピーしてビルド (/mnt/c 上のビルドは遅いため)
rm -rf "$WS/src/diffbot_task"
cp -r "$HERE/ros2_ws/src/diffbot_task" "$WS/src/"
source /opt/ros/humble/setup.bash
cd "$WS" && colcon build --symlink-install
source "$WS/install/setup.bash"

RAW=/tmp/patrol_raw.mp4
rm -f $RAW
# 万一止まらなくても 20 分で打ち切る
timeout -s INT 1200 ros2 launch diffbot_task patrol.launch.py output:=$RAW "${@:2}" || true
[ -s $RAW ] || { echo "録画ファイルができていません"; exit 1; }

ffmpeg -y -loglevel error -i $RAW -c:v libx264 -pix_fmt yuv420p -crf 20 "$OUT_DIR/patrol_demo.mp4"
echo "saved: $OUT_DIR/patrol_demo.mp4"
ffprobe -v error -show_entries format=duration -of csv=p=0 "$OUT_DIR/patrol_demo.mp4"
