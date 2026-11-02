#!/bin/bash
# 学習済み方策の走行だけを録画する (run_pipeline.sh の手順 6 を単独で実行)
# 実行: bash record_ai_demo.sh
set -e
HERE="$(cd "$(dirname "$0")/.." && pwd)"
OUT_DIR="$HERE/output"
CKPT=${CKPT:-~/checkpoints/patrol_bc_dagger}   # 既定は DAgger 後の方策
source /opt/ros/humble/setup.bash
source ~/diffbot_ws/install/setup.bash
RAW=~/patrol_ai_raw.mp4   # /tmp は WSL 再起動で消えるのでホームに置く
rm -f $RAW
timeout -s INT 900 ros2 launch diffbot_task patrol.launch.py controller:=policy \
  model_path:=$CKPT/policy.npz output:=$RAW 2>&1 \
  | grep --line-buffered -E "(patrol_node|video_recorder).*(到達|timeout|巡回完了|録画)" || true
[ -s $RAW ] || { echo "録画ファイルができていません"; exit 1; }
ffmpeg -y -loglevel error -i $RAW -c:v libx264 -pix_fmt yuv420p -crf 20 "$OUT_DIR/patrol_ai_demo.mp4"
echo "saved: $OUT_DIR/patrol_ai_demo.mp4"
ffprobe -v error -show_entries format=duration -of csv=p=0 "$OUT_DIR/patrol_ai_demo.mp4"
