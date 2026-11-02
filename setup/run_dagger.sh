#!/bin/bash
# DAgger (Dataset Aggregation) を 1 反復まわす
#   1. 現在の方策 (patrol_bc) でランダムな目標を走らせ、お手本のラベルつきで記録 (dagger*.bag)
#   2. お手本のデータ + DAgger データをまとめて変換
#   3. 学習し直す → ~/checkpoints/patrol_bc_dagger
# 実行: bash run_dagger.sh collect 0 1     (seed 10, 11 の DAgger 収集。10 分以内に収まるよう分割)
#       bash run_dagger.sh train           (変換・学習・デバッグ)
set -e
HERE="$(cd "$(dirname "$0")/.." && pwd)"
DATA=~/dataset
CKPT0=~/checkpoints/patrol_bc
CKPT1=~/checkpoints/patrol_bc_dagger
source /opt/ros/humble/setup.bash
source ~/diffbot_ws/install/setup.bash

if [ "$1" = "collect" ]; then
  shift
  for i in "$@"; do
    echo "--- dagger$i seed=$((10 + i))"
    rm -rf $DATA/rosbags/dagger$i
    timeout -s INT 500 ros2 launch diffbot_task collect.launch.py bag:=$DATA/rosbags/dagger$i \
      seed:=$((10 + i)) n_goals:=10 noise:=false controller:=policy model_path:=$CKPT0/policy.npz \
      goal_timeout:=30 2>&1 | grep --line-buffered -E "patrol_node.*(timeout|巡回完了)" \
      | sed -u 's/\[INFO\] \[[0-9.]*\] //' || true
  done
elif [ "$1" = "train" ]; then
  ros2 run diffbot_task rosbag2dataset --bags $DATA/rosbags --out $DATA/patrol_bc_dagger.npz | grep -E "bags:|saved|dagger.*timeout" || true
  ros2 run diffbot_task train_bc --data $DATA/patrol_bc_dagger.npz --out $CKPT1 | tail -3
  ros2 run diffbot_task debug_policy --data $DATA/patrol_bc_dagger.npz --ckpt $CKPT1 | head -2
  D="$HERE/output/ai_results/dagger"
  mkdir -p "$D"
  cp $CKPT1/policy.npz $CKPT1/policy.pt $CKPT1/loss_curve.png $CKPT1/train_log.txt \
     $CKPT1/debug_policy.png $CKPT1/debug_policy.txt "$D/"
fi
