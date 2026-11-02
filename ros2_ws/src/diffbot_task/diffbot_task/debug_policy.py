"""方策のデバッグ (第5回講義の debug-policy に相当)。

検証用エピソードの観測を学習済み方策に入れ、推論した行動とお手本の行動 (Ground Truth) を比べる。
「学習データ上で正しく振る舞えるか」を先に確かめてから、デプロイ時の汎化を疑うための道具。

  ros2 run diffbot_task debug_policy --data ~/dataset/patrol_bc.npz --ckpt ~/checkpoints/patrol_bc
"""
import argparse
import os

import numpy as np

from diffbot_task import features as F


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', default=os.path.expanduser('~/dataset/patrol_bc.npz'))
    ap.add_argument('--ckpt', default=os.path.expanduser('~/checkpoints/patrol_bc'))
    a = ap.parse_args()
    d = np.load(a.data)
    obs, act, ep = d['obs'], d['act'], d['episode']
    val_eps = np.load(os.path.join(a.ckpt, 'val_episodes.npy'))
    policy = F.NumpyMLP(os.path.join(a.ckpt, 'policy.npz'))
    m = np.isin(ep, val_eps)
    pred = np.array([policy(o) for o in obs[m]])
    gt = act[m]
    lines = []
    for k, name, scale, unit in ((0, 'linear v', F.V_MAX, 'm/s'), (1, 'angular w', F.W_MAX, 'rad/s')):
        mae = np.abs(pred[:, k] - gt[:, k]).mean() * scale
        r2 = 1 - ((pred[:, k] - gt[:, k]) ** 2).sum() / ((gt[:, k] - gt[:, k].mean()) ** 2).sum()
        lines.append(f'{name}: MAE {mae:.3f} {unit}, R2 {r2:.3f}')
    print('\n'.join(lines))
    with open(os.path.join(a.ckpt, 'debug_policy.txt'), 'w') as f:
        f.write(f'validation frames: {m.sum()}, episodes: {len(val_eps)}\n' + '\n'.join(lines) + '\n')

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    # 検証エピソードのうち最も長いものを表示
    lens = {e: (ep == e).sum() for e in val_eps}
    e = max(lens, key=lens.get)
    sel = ep == e
    p = np.array([policy(o) for o in obs[sel]])
    t = np.arange(sel.sum()) / 10.0
    fig, ax = plt.subplots(2, 1, figsize=(6.4, 4.2), sharex=True)
    for k, (lab, sc, un) in enumerate((('v', F.V_MAX, 'm/s'), ('omega', F.W_MAX, 'rad/s'))):
        ax[k].plot(t, act[sel, k] * sc, label='expert (GT)', lw=2)
        ax[k].plot(t, p[:, k] * sc, '--', label='learned policy', lw=1.6)
        ax[k].set_ylabel(f'{lab} [{un}]')
        ax[k].grid(alpha=0.3)
    ax[0].legend(loc='upper right', fontsize=8)
    ax[1].set_xlabel('time in episode [s]')
    fig.suptitle(f'Policy vs expert on a validation episode (ep {e})')
    fig.tight_layout()
    fig.savefig(os.path.join(a.ckpt, 'debug_policy.png'), dpi=150)
    print('saved debug_policy.png')


if __name__ == '__main__':
    main()
