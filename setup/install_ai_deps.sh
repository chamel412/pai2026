#!/bin/bash
# 模倣学習 (学習側) に必要なものを入れる: PyTorch (CPU 版) と matplotlib
# 実行: sudo bash install_ai_deps.sh
set -e
export DEBIAN_FRONTEND=noninteractive
apt-get install -y python3-pip python3-matplotlib
# CPU 版 PyTorch (GPU 不要・WSL でもそのまま動く)
pip3 install --no-cache-dir torch --index-url https://download.pytorch.org/whl/cpu
# torch と一緒に入る新しい setuptools は colcon (ament_python) のビルドを壊すので外し、apt 版に戻す
pip3 uninstall -y setuptools || true
python3 -c "import torch, numpy; print('torch', torch.__version__, 'numpy', numpy.__version__)"
echo "ai deps done"
