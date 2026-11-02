"""模倣学習 (Behavior Cloning): お手本の (観測 → 行動) を MLP に学習させる。

  ros2 run diffbot_task train_bc --data ~/dataset/patrol_bc.npz --out ~/checkpoints/patrol_bc

- エピソード単位で学習用 80% / 検証用 20% に分ける (同じエピソードのフレームが両方に入らないように)
- 観測は学習データの平均・標準偏差で標準化
- MLP: 39 → 256 → 256 → 2 (ReLU, 出力は tanh で -1〜1)、損失は MSE
- 出力: policy.npz (numpy で推論できる重み)、policy.pt、loss_curve.png、train_log.txt
"""
import argparse
import os

import numpy as np
import torch
from torch import nn

from diffbot_task import features as F


def build(hidden=256):
    return nn.Sequential(
        nn.Linear(F.OBS_DIM, hidden), nn.ReLU(),
        nn.Linear(hidden, hidden), nn.ReLU(),
        nn.Linear(hidden, F.ACT_DIM), nn.Tanh())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', default=os.path.expanduser('~/dataset/patrol_bc.npz'))
    ap.add_argument('--out', default=os.path.expanduser('~/checkpoints/patrol_bc'))
    ap.add_argument('--epochs', type=int, default=200)
    ap.add_argument('--batch', type=int, default=256)
    ap.add_argument('--lr', type=float, default=1e-3)
    ap.add_argument('--seed', type=int, default=0)
    a = ap.parse_args()
    torch.manual_seed(a.seed)
    rng = np.random.default_rng(a.seed)
    os.makedirs(a.out, exist_ok=True)

    d = np.load(a.data)
    obs, act, ep = d['obs'].astype(np.float32), d['act'].astype(np.float32), d['episode']
    eps = np.unique(ep)
    rng.shuffle(eps)
    n_val = max(1, int(len(eps) * 0.2))
    val_eps = set(eps[:n_val].tolist())
    is_val = np.array([e in val_eps for e in ep])
    mean = obs[~is_val].mean(0)
    std = np.maximum(obs[~is_val].std(0), 1e-3)
    norm = lambda x: torch.from_numpy((x - mean) / std)
    Xtr, Ytr = norm(obs[~is_val]), torch.from_numpy(act[~is_val])
    Xva, Yva = norm(obs[is_val]), torch.from_numpy(act[is_val])

    model = build()
    opt = torch.optim.Adam(model.parameters(), lr=a.lr, weight_decay=1e-5)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, a.epochs)
    loss_fn = nn.MSELoss()
    log = [f'data: {len(obs)} frames, {len(eps)} episodes  train {len(Xtr)} / val {len(Xva)} frames '
           f'({len(eps) - n_val} / {n_val} episodes)']
    print(log[0])
    hist, best, best_state = [], float('inf'), None
    for epoch in range(1, a.epochs + 1):
        model.train()
        perm = torch.randperm(len(Xtr))
        tot = 0.0
        for i in range(0, len(Xtr), a.batch):
            idx = perm[i:i + a.batch]
            loss = loss_fn(model(Xtr[idx]), Ytr[idx])
            opt.zero_grad()
            loss.backward()
            opt.step()
            tot += loss.item() * len(idx)
        sched.step()
        model.eval()
        with torch.no_grad():
            vl = loss_fn(model(Xva), Yva).item()
        hist.append((tot / len(Xtr), vl))
        if vl < best:
            best, best_state = vl, {k: v.clone() for k, v in model.state_dict().items()}
        if epoch % 20 == 0 or epoch == 1:
            line = f'epoch {epoch:4d}  train {tot / len(Xtr):.5f}  val {vl:.5f}'
            print(line)
            log.append(line)

    model.load_state_dict(best_state)
    torch.save(model.state_dict(), os.path.join(a.out, 'policy.pt'))
    lin = [m for m in model if isinstance(m, nn.Linear)]
    np.savez(os.path.join(a.out, 'policy.npz'), obs_mean=mean, obs_std=std,
             **{f'W{i}': m.weight.detach().numpy() for i, m in enumerate(lin)},
             **{f'b{i}': m.bias.detach().numpy() for i, m in enumerate(lin)})
    log.append(f'best val MSE {best:.5f}')
    with open(os.path.join(a.out, 'train_log.txt'), 'w') as f:
        f.write('\n'.join(log) + '\n')
    np.save(os.path.join(a.out, 'val_episodes.npy'), np.array(sorted(val_eps)))

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    h = np.array(hist)
    plt.figure(figsize=(6, 3.6))
    plt.plot(h[:, 0], label='train')
    plt.plot(h[:, 1], label='validation')
    plt.yscale('log')
    plt.xlabel('epoch')
    plt.ylabel('MSE (normalized action)')
    plt.title('Behavior cloning loss')
    plt.legend()
    plt.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(a.out, 'loss_curve.png'), dpi=150)
    print(f'best val MSE {best:.5f} → saved to {a.out}')


if __name__ == '__main__':
    main()
