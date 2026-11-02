#!/bin/bash
# 模倣学習パイプラインを最初から最後まで実行する (第5回講義の 収集→変換→学習→デプロイ→評価 の流れ)
#   1. お手本 (ポテンシャル法) でデモを収集 (rosbag2)       … N_RUNS 回 × N_GOALS 目標
#   2. rosbag2 → 学習用データセット (10 Hz, 直前値保持)
#   3. 模倣学習 (MLP, PyTorch)
#   4. 方策のデバッグ (検証データで お手本 vs 方策)
#   5. 評価: お手本 と 学習済み方策 で同じ巡回ルート A→B→C→H を走らせて比較
#   6. 学習済み方策の走行を録画 → H.264 mp4
# 実行: bash run_pipeline.sh            (すべて)
#       SKIP_COLLECT=1 bash run_pipeline.sh   (収集済みデータを使い、変換以降だけ)
set -e
HERE="$(cd "$(dirname "$0")/.." && pwd)"
OUT_DIR="$HERE/output"
N_RUNS=${N_RUNS:-4}
N_GOALS=${N_GOALS:-12}
DATA=~/dataset
CKPT=~/checkpoints/patrol_bc
mkdir -p "$OUT_DIR/ai_results"

bash "$HERE/setup/build.sh"
source /opt/ros/humble/setup.bash
source ~/diffbot_ws/install/setup.bash

if [ -z "$SKIP_COLLECT" ]; then
  echo "===== 1. デモ収集 ($N_RUNS runs x $N_GOALS goals) ====="
  rm -rf $DATA/rosbags && mkdir -p $DATA/rosbags
  for i in $(seq 0 $((N_RUNS - 1))); do
    # 半分はノイズなし (きれいなお手本)、半分はノイズあり (ずれた状態からの立て直し)
    NOISE=$([ $((i % 2)) -eq 1 ] && echo true || echo false)
    echo "--- run$i seed=$i noise=$NOISE"
    timeout -s INT 900 ros2 launch diffbot_task collect.launch.py \
      bag:=$DATA/rosbags/run$i seed:=$i n_goals:=$N_GOALS noise:=$NOISE 2>&1 \
      | grep -E "patrol_node.*(到達|timeout|巡回完了|controller)" || true
  done
fi

echo "===== 2. rosbag2 → データセット ====="
ros2 run diffbot_task rosbag2dataset --bags $DATA/rosbags --out $DATA/patrol_bc.npz | tail -3

echo "===== 3. 模倣学習 ====="
ros2 run diffbot_task train_bc --data $DATA/patrol_bc.npz --out $CKPT

echo "===== 4. 方策のデバッグ ====="
ros2 run diffbot_task debug_policy --data $DATA/patrol_bc.npz --ckpt $CKPT

echo "===== 5. 評価 (お手本 / 学習済み方策, 録画なし) ====="
for C in expert policy; do
  timeout -s INT 600 ros2 launch diffbot_task patrol.launch.py controller:=$C record:=false \
    model_path:=$CKPT/policy.npz 2>&1 | grep -E "patrol_node.*(到達|timeout|巡回完了)" | tee "$OUT_DIR/ai_results/eval_$C.txt" || true
done

echo "===== 6. 学習済み方策の走行を録画 ====="
RAW=/tmp/patrol_ai_raw.mp4
rm -f $RAW
timeout -s INT 1200 ros2 launch diffbot_task patrol.launch.py controller:=policy \
  model_path:=$CKPT/policy.npz output:=$RAW 2>&1 | grep -E "(patrol_node|video_recorder).*(到達|timeout|巡回完了|録画)" || true
[ -s $RAW ] || { echo "録画ファイルができていません"; exit 1; }
ffmpeg -y -loglevel error -i $RAW -c:v libx264 -pix_fmt yuv420p -crf 20 "$OUT_DIR/patrol_ai_demo.mp4"

cp $CKPT/policy.npz $CKPT/policy.pt $CKPT/loss_curve.png $CKPT/train_log.txt \
   $CKPT/debug_policy.png $CKPT/debug_policy.txt "$OUT_DIR/ai_results/"
echo "saved: $OUT_DIR/patrol_ai_demo.mp4"
ffprobe -v error -show_entries format=duration -of csv=p=0 "$OUT_DIR/patrol_ai_demo.mp4"
