"""rosbag2 → 学習用データセット (npz) への変換 (第5回講義の rosbag2lerobot に相当)。

rosbag2 の各トピックは非同期 (LiDAR 10 Hz, オドメトリ 30 Hz, 指令 20 Hz) なので、
エピソードごとに一定周期 (既定 10 Hz) の時刻列を作り、各時刻について
「その時刻以前で一番新しい値」を使って (直前値保持) 観測と行動をそろえる。

  ros2 run diffbot_task rosbag2dataset --bags ~/dataset/rosbags --out ~/dataset/patrol_bc.npz

お手本が走ったエピソードは、成功したもの (目標に到達できたもの) だけを使う。時間切れの
エピソードは「止まる・迷う」動きを学習させないために捨てる (データの質が方策の質を決める)。
DAgger の bag (名前が dagger で始まるもの) は方策が走り、ラベルはお手本の指令なので、
方策が行き詰まった (時間切れの) エピソードも残す。そこが一番学ばせたい状態だから。
"""
import argparse
import bisect
import glob
import math
import os

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

from diffbot_task import features as F


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    scans, odoms, experts, goals, results = [], [], [], {}, {}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in types:
            continue
        msg = deserialize_message(data, types[topic])
        if topic == '/scan':
            scans.append((stamp(msg), F.lidar_sectors(msg.ranges, msg.angle_min, msg.angle_increment)))
        elif topic == '/odom':
            p = msg.pose.pose
            odoms.append((stamp(msg), p.position.x, p.position.y, yaw_from_quat(p.orientation)))
        elif topic == '/expert_cmd':
            experts.append((stamp(msg), msg.header.frame_id, msg.twist.linear.x, msg.twist.angular.z))
        elif topic == '/patrol/goal':
            goals[msg.header.frame_id] = (msg.point.x, msg.point.y)
        elif topic == '/patrol/episode':
            ep, res = msg.data.split()
            results[ep] = res
    for lst in (scans, odoms, experts):
        lst.sort(key=lambda r: r[0])
    return scans, odoms, experts, goals, results


def zoh(times, rows, t):
    """時刻 t 以前で一番新しい行 (直前値保持)。なければ None"""
    i = bisect.bisect_right(times, t) - 1
    return rows[i] if i >= 0 else None


def convert(bag_dirs, fps):
    obs, act, ep_ids, n_skip = [], [], [], 0
    ep_counter = 0
    for bag in bag_dirs:
        scans, odoms, experts, goals, results = read_bag(bag)
        dagger = os.path.basename(bag).startswith('dagger')
        st, ot, et = [r[0] for r in scans], [r[0] for r in odoms], [r[0] for r in experts]
        for ep in sorted(results, key=lambda e: int(e[2:])):
            if (results[ep] != 'success' and not dagger) or ep not in goals:
                n_skip += 1
                continue
            ts = [r[0] for r in experts if r[1] == ep]
            if len(ts) < 5:
                continue
            gx, gy = goals[ep]
            n = 0
            for t in np.arange(ts[0], ts[-1], 1.0 / fps):
                s, o, e = zoh(st, scans, t), zoh(ot, odoms, t), zoh(et, experts, t)
                if s is None or o is None or e is None or e[1] != ep:
                    continue
                obs.append(F.make_obs(s[1], o[1], o[2], o[3], gx, gy))
                act.append(F.act_to_norm(e[2], e[3]))
                ep_ids.append(ep_counter)
                n += 1
            print(f'  {os.path.basename(bag)} {ep} ({results[ep]}): {n} frames')
            ep_counter += 1
    return np.array(obs), np.array(act), np.array(ep_ids), ep_counter, n_skip


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bags', default=os.path.expanduser('~/dataset/rosbags'))
    ap.add_argument('--out', default=os.path.expanduser('~/dataset/patrol_bc.npz'))
    ap.add_argument('--fps', type=float, default=10.0)
    a = ap.parse_args()
    bag_dirs = sorted(d for d in glob.glob(os.path.join(a.bags, '*')) if os.path.exists(os.path.join(d, 'metadata.yaml')))
    print(f'{len(bag_dirs)} bags: {[os.path.basename(b) for b in bag_dirs]}')
    obs, act, ep, n_ep, n_skip = convert(bag_dirs, a.fps)
    os.makedirs(os.path.dirname(a.out) or '.', exist_ok=True)
    np.savez_compressed(a.out, obs=obs, act=act, episode=ep)
    print(f'saved {a.out}: {len(obs)} frames, {n_ep} episodes (skipped {n_skip} failed), '
          f'obs {obs.shape}, act {act.shape}')


if __name__ == '__main__':
    main()
