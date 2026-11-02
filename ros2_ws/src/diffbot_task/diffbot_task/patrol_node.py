"""巡回ノード: LiDAR とオドメトリで目標地点を順に回る。制御器を 2 種類切り替えられる。

  controller:=expert  … 手で設計したポテンシャル法 (模倣学習の「お手本」)
  controller:=policy  … お手本を模倣学習したニューラルネット (AI) が速度指令を出す

入力 : /odom, /scan
出力 : /cmd_vel           実際にロボットへ送る指令
       /expert_cmd        その時点でお手本が出す指令 (TwistStamped, 学習の教師ラベル)
       /patrol/goal       現在の目標 (PointStamped, frame_id = エピソード ID)
       /patrol/episode    エピソード結果 ("ep3 success" / "ep3 timeout")
       /patrol/status     動画表示用の状態文字列

データ収集用オプション
  random_goals:=true  … 空いている場所にランダムな目標を n_random_goals 個生成して回る
  noise:=true         … 実行する指令にだけ時間相関ノイズを加える (ラベルはノイズなし)
                         → お手本の軌道から少しずれた状態とその「立て直し方」も記録でき、
                           模倣学習で起きやすい誤差の蓄積 (共変量シフト) を減らす (DART 方式)

お手本の制御方式 (ポテンシャル法 + 回り込み項):
  進みたい方向 = 目標への引力 + LiDAR 各ビームからの斥力 + 正面に障害物があるときの回り込み
  その方向へ P 制御で旋回し、向きのずれと前方距離に応じて減速する。
"""
import math
import random

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from diffbot_task import features as F
from diffbot_task.arena import ARENA, OBSTACLES, WAYPOINTS


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def obstacle_clearance(x, y):
    """点 (x, y) から最も近い障害物表面までのおおよその距離"""
    best = float('inf')
    for ob in OBSTACLES:
        if ob[0] == 'cyl':
            best = min(best, math.hypot(x - ob[1], y - ob[2]) - ob[3])
        else:
            _, cx, cy, sx, sy, yaw = ob
            lx = (x - cx) * math.cos(yaw) + (y - cy) * math.sin(yaw)
            ly = -(x - cx) * math.sin(yaw) + (y - cy) * math.cos(yaw)
            ddx, ddy = max(abs(lx) - sx / 2, 0.0), max(abs(ly) - sy / 2, 0.0)
            best = min(best, math.hypot(ddx, ddy))
    return best


def random_goals(n, seed, start=(0.0, 0.0)):
    rng = random.Random(seed)
    xmin, xmax, ymin, ymax = ARENA
    goals, prev = [], start
    while len(goals) < n:
        x = rng.uniform(xmin + 0.7, xmax - 0.7)
        y = rng.uniform(ymin + 0.7, ymax - 0.7)
        if obstacle_clearance(x, y) < 0.6 or math.hypot(x - prev[0], y - prev[1]) < 2.0:
            continue
        goals.append((f'R{len(goals) + 1}', round(x, 2), round(y, 2)))
        prev = (x, y)
    return goals


class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')
        self.declare_parameter('controller', 'expert')   # expert / policy
        self.declare_parameter('model_path', '')
        self.declare_parameter('random_goals', False)
        self.declare_parameter('n_random_goals', 12)
        self.declare_parameter('seed', 0)
        self.declare_parameter('noise', False)
        self.declare_parameter('noise_v', 0.08)          # [m/s]
        self.declare_parameter('noise_w', 0.6)           # [rad/s]
        self.declare_parameter('goal_timeout', 45.0)     # [s] これを超えたらエピソード失敗
        self.declare_parameter('max_linear', F.V_MAX)    # [m/s]
        self.declare_parameter('max_angular', F.W_MAX)   # [rad/s]
        self.declare_parameter('goal_tolerance', 0.18)   # [m]
        self.declare_parameter('influence_dist', 0.9)    # 斥力が働き始める距離 [m]
        self.declare_parameter('k_repulse', 2.0)
        self.declare_parameter('k_tangent', 1.6)
        self.declare_parameter('pause_sec', 1.5)
        self.declare_parameter('start_delay', 3.0)
        self.declare_parameter('exit_on_done', False)    # 収集時: 巡回完了でノードを終了
        self.finished = False

        self.controller = self.get_parameter('controller').value
        if self.get_parameter('random_goals').value:
            self.waypoints = random_goals(self.get_parameter('n_random_goals').value,
                                          self.get_parameter('seed').value)
        else:
            self.waypoints = list(WAYPOINTS)
        self.policy = None
        if self.controller == 'policy':
            self.policy = F.NumpyMLP(self.get_parameter('model_path').value)
            self.get_logger().info(f'学習済み方策を読み込み: {self.get_parameter("model_path").value}')
        self.get_logger().info(f'controller={self.controller}  目標: '
                               + ' '.join(f'{n}({x:.1f},{y:.1f})' for n, x, y in self.waypoints))

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.expert_pub = self.create_publisher(TwistStamped, '/expert_cmd', 10)
        self.goal_pub = self.create_publisher(PointStamped, '/patrol/goal', 10)
        self.episode_pub = self.create_publisher(String, '/patrol/episode', 10)
        self.status_pub = self.create_publisher(String, '/patrol/status', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)

        self.pose = None
        self.scan = None
        self.state = 'WAIT'
        self.idx = 0
        self.t_state = None
        self.t_start = None
        self.t_goal = None
        self.side = 0
        self.distance = 0.0
        self.last_xy = None
        self.min_clear = float('inf')   # 走行中の LiDAR 最小距離 (障害物への最接近)
        self.n_success = 0
        self.n_timeout = 0
        self.noise_state = np.zeros(2)
        self.timer = self.create_timer(0.05, self.step)  # 20 Hz

    # ---------- コールバック ----------
    def on_odom(self, msg):
        p = msg.pose.pose
        x, y = p.position.x, p.position.y
        if self.last_xy is not None and self.state == 'GOTO':
            self.distance += math.hypot(x - self.last_xy[0], y - self.last_xy[1])
        self.last_xy = (x, y)
        self.pose = (x, y, yaw_from_quat(p.orientation))

    def on_scan(self, msg):
        self.scan = msg
        if self.state == 'GOTO':
            valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
            if valid:
                self.min_clear = min(self.min_clear, min(valid))

    # ---------- 補助 ----------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def publish_status(self, text):
        self.status_pub.publish(String(data=text))

    def sector_min(self, lo, hi):
        s = self.scan
        best = float('inf')
        for i, r in enumerate(s.ranges):
            a = s.angle_min + i * s.angle_increment
            if lo <= a <= hi and s.range_min < r < s.range_max:
                best = min(best, r)
        return best

    def sector_mean(self, lo, hi):
        s = self.scan
        vals = []
        for i, r in enumerate(s.ranges):
            a = s.angle_min + i * s.angle_increment
            if lo <= a <= hi:
                vals.append(r if math.isfinite(r) and r < s.range_max else s.range_max)
        return sum(vals) / len(vals) if vals else 0.0

    def stop(self):
        self.cmd_pub.publish(Twist())

    def end_episode(self, t, result):
        name = self.waypoints[self.idx][0]
        self.episode_pub.publish(String(data=f'ep{self.idx} {result}'))
        if result == 'success':
            self.n_success += 1
            self.get_logger().info(f'{name} に到達 (t={t - self.t_start:.1f}s)')
        else:
            self.n_timeout += 1
            self.get_logger().warn(f'{name} に {result} (t={t - self.t_start:.1f}s)')
        self.side = 0
        self.noise_state[:] = 0.0
        if self.idx == len(self.waypoints) - 1:
            self.state, self.t_done = 'DONE', t
            self.get_logger().info(
                f'巡回完了: {self.t_done - self.t_start:.1f}s, {self.distance:.1f}m, '
                f'最接近 {self.min_clear:.2f}m, 成功 {self.n_success}/{len(self.waypoints)}')
        else:
            self.state, self.t_state = 'PAUSE', t

    # ---------- お手本 (ポテンシャル法) ----------
    def expert_command(self, gx, gy):
        x, y, yaw = self.pose
        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)
        d0 = self.get_parameter('influence_dist').value
        vx, vy = dx / dist, dy / dist
        gvx, gvy = vx, vy

        # 斥力: 各ビームについて (1/r - 1/d0) の大きさで障害物と逆向きに押す
        s = self.scan
        rx = ry = 0.0
        for i, r in enumerate(s.ranges):
            if not (s.range_min < r < d0):
                continue
            a = yaw + s.angle_min + i * s.angle_increment
            w = (1.0 / r - 1.0 / d0) * s.angle_increment
            rx -= w * math.cos(a)
            ry -= w * math.sin(a)
        k_rep = self.get_parameter('k_repulse').value
        vx += k_rep * rx
        vy += k_rep * ry

        # 回り込み項: 目標方向の正面 ±35° に障害物があるとき、空いている側へ横向きの力
        goal_rel = wrap(math.atan2(dy, dx) - yaw)
        front = self.sector_min(goal_rel - 0.6, goal_rel + 0.6)
        blocked = front < d0 and front < dist
        if blocked:
            if self.side == 0:
                left = self.sector_mean(goal_rel + 0.3, goal_rel + 1.6)
                right = self.sector_mean(goal_rel - 1.6, goal_rel - 0.3)
                self.side = 1 if left >= right else -1
                if self.controller == 'expert':
                    self.get_logger().info(f'障害物検知 → {"左" if self.side > 0 else "右"}から回り込み')
            k_t = self.get_parameter('k_tangent').value * (1.0 - front / d0)
            vx += k_t * (-gvy * self.side)
            vy += k_t * (gvx * self.side)
        elif front > d0 * 1.3:
            self.side = 0
        avoiding = blocked or math.hypot(rx, ry) * k_rep > 0.3

        err = wrap(math.atan2(vy, vx) - yaw)
        vmax = self.get_parameter('max_linear').value
        wmax = self.get_parameter('max_angular').value
        ahead = self.sector_min(-0.5, 0.5)
        speed_scale = min(1.0, max(0.0, (ahead - 0.22) / 0.5))
        lin = vmax * max(0.0, math.cos(err)) ** 2 * speed_scale
        lin = min(lin, 0.6 * dist + 0.05)
        ang = max(-wmax, min(wmax, 2.2 * err))
        return lin, ang, avoiding

    # ---------- メインループ ----------
    def step(self):
        t = self.now()
        if self.pose is None or self.scan is None:
            return
        if self.state == 'WAIT':
            if self.t_state is None:
                self.t_state = t
                self.get_logger().info('センサ受信OK。巡回を開始します')
            self.publish_status('READY')
            if t - self.t_state >= self.get_parameter('start_delay').value:
                self.state, self.t_start, self.t_goal = 'GOTO', t, t
            return
        if self.state == 'PAUSE':
            self.stop()
            self.publish_status(f'ARRIVED {self.waypoints[self.idx][0]}')
            if t - self.t_state >= self.get_parameter('pause_sec').value:
                self.idx += 1
                self.state, self.t_goal = 'GOTO', t
            return
        if self.state == 'DONE':
            self.stop()
            if self.get_parameter('exit_on_done').value and t - self.t_done > 1.0:
                self.finished = True
            self.publish_status(f'DONE time {self.t_done - self.t_start:.1f}s dist {self.distance:.1f}m '
                                f'clear {self.min_clear:.2f}m ok {self.n_success}/{len(self.waypoints)}')
            return

        # ---- GOTO ----
        name, gx, gy = self.waypoints[self.idx]
        x, y, yaw = self.pose
        dist = math.hypot(gx - x, gy - y)
        if dist < self.get_parameter('goal_tolerance').value:
            self.stop()
            self.end_episode(t, 'success')
            return
        if t - self.t_goal > self.get_parameter('goal_timeout').value:
            self.stop()
            self.end_episode(t, 'timeout')
            return

        stamp = self.get_clock().now().to_msg()
        ep = f'ep{self.idx}'
        goal = PointStamped()
        goal.header.stamp, goal.header.frame_id = stamp, ep
        goal.point.x, goal.point.y = float(gx), float(gy)
        self.goal_pub.publish(goal)

        # お手本の指令は常に計算して配信する (収集時は教師ラベル、デプロイ時は比較用)
        e_lin, e_ang, avoiding = self.expert_command(gx, gy)
        ex = TwistStamped()
        ex.header.stamp, ex.header.frame_id = stamp, ep
        ex.twist.linear.x, ex.twist.angular.z = e_lin, e_ang
        self.expert_pub.publish(ex)

        if self.controller == 'policy':
            sectors = F.lidar_sectors(self.scan.ranges, self.scan.angle_min, self.scan.angle_increment)
            obs = F.make_obs(sectors, x, y, yaw, gx, gy)
            lin, ang = F.norm_to_act(self.policy(obs))
            mode = 'AI'
        else:
            lin, ang = e_lin, e_ang
            mode = 'AVOID' if avoiding else 'GO'

        if self.get_parameter('noise').value:
            # 時間相関のあるノイズ (1 次遅れ) を実行指令にだけ加える
            sig = np.array([self.get_parameter('noise_v').value, self.get_parameter('noise_w').value])
            self.noise_state = 0.9 * self.noise_state + 0.1 * np.random.randn(2) * sig * 4.4
            lin = float(np.clip(lin + self.noise_state[0], 0.0, F.V_MAX))
            ang = float(np.clip(ang + self.noise_state[1], -F.W_MAX, F.W_MAX))

        cmd = Twist()
        cmd.linear.x = lin
        cmd.angular.z = ang
        self.cmd_pub.publish(cmd)
        self.publish_status(f'{mode} {name} {dist:.2f}')


def main():
    rclpy.init()
    node = PatrolNode()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
