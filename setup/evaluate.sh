#!/bin/bash
# 評価: 巡回ルート A→B→C→H を指定の制御器で N 回走らせ、結果を 1 行ずつ追記する (録画なし)
# 実行: bash evaluate.sh policy ~/checkpoints/patrol_bc_dagger/policy.npz 3 [結果ファイル]
#       bash evaluate.sh expert - 1
set -e
HERE="$(cd "$(dirname "$0")/.." && pwd)"
C=$1; MODEL=${2:--}; N=${3:-3}
OUT=${4:-$HERE/output/ai_results/evaluation.txt}
mkdir -p "$(dirname "$OUT")"
source /opt/ros/humble/setup.bash
source ~/diffbot_ws/install/setup.bash
for k in $(seq 1 $N); do
  R=$(timeout -s INT 300 ros2 launch diffbot_task patrol.launch.py controller:=$C record:=false \
        model_path:=$MODEL 2>&1 | grep -E "patrol_node.*巡回完了" | sed 's/.*巡回完了: //')
  echo "$C $(basename $(dirname $MODEL)) trial$k: ${R:-(no result)}" | tee -a "$OUT"
done
