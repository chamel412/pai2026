"""模倣学習で使う「観測」と「行動」の定義。

データ変換 (rosbag2dataset)・学習 (train_bc)・デプロイ (patrol_node の policy モード) の
3 か所で同じ関数を使い、学習時とデプロイ時で観測の作り方がずれないようにする。

観測 o (39 次元)
  - LiDAR 360 本を 10° ごとの 36 セクタに分け、各セクタの最小距離を 0〜1 に正規化 (3 m で頭打ち)
  - 目標までの距離 (3 m で頭打ち, 0〜1)
  - 目標方向 (ロボット座標) の sin, cos
行動 a (2 次元)
  - 並進速度 / V_MAX, 角速度 / W_MAX  (どちらも -1〜1)
"""
import math

import numpy as np

N_SECTORS = 36
RANGE_CLIP = 3.0
GOAL_CLIP = 3.0
V_MAX = 0.35
W_MAX = 1.5
OBS_DIM = N_SECTORS + 3
ACT_DIM = 2


def lidar_sectors(ranges, angle_min, angle_increment):
    """LiDAR 距離列 → 36 セクタの最小距離 (0〜1)。inf / nan は最大距離とみなす。"""
    r = np.asarray(ranges, dtype=np.float32)
    r = np.where(np.isfinite(r), r, RANGE_CLIP)
    r = np.clip(r, 0.0, RANGE_CLIP)
    ang = angle_min + np.arange(len(r)) * angle_increment
    idx = ((ang + math.pi) / (2 * math.pi) * N_SECTORS).astype(int) % N_SECTORS
    out = np.full(N_SECTORS, RANGE_CLIP, dtype=np.float32)
    np.minimum.at(out, idx, r)
    return out / RANGE_CLIP


def make_obs(sectors, x, y, yaw, gx, gy):
    dx, dy = gx - x, gy - y
    dist = math.hypot(dx, dy)
    rel = math.atan2(dy, dx) - yaw
    return np.concatenate([
        sectors,
        np.array([min(dist, GOAL_CLIP) / GOAL_CLIP, math.sin(rel), math.cos(rel)], dtype=np.float32),
    ]).astype(np.float32)


def act_to_norm(v, w):
    return np.array([v / V_MAX, w / W_MAX], dtype=np.float32)


def norm_to_act(a):
    return float(np.clip(a[0], 0.0, 1.0) * V_MAX), float(np.clip(a[1], -1.0, 1.0) * W_MAX)


class NumpyMLP:
    """学習済み MLP を numpy だけで推論する (デプロイ側に PyTorch を不要にするため)。

    重みファイル (npz): W0,b0,W1,b1,...  + obs_mean, obs_std
    隠れ層は ReLU、出力層は tanh。
    """

    def __init__(self, path):
        d = np.load(path)
        self.layers = []
        i = 0
        while f'W{i}' in d:
            self.layers.append((d[f'W{i}'], d[f'b{i}']))
            i += 1
        self.mean = d['obs_mean']
        self.std = d['obs_std']

    def __call__(self, obs):
        h = (obs - self.mean) / self.std
        for k, (W, b) in enumerate(self.layers):
            h = h @ W.T + b
            h = np.maximum(h, 0.0) if k < len(self.layers) - 1 else np.tanh(h)
        return h
