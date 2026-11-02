"""動画記録ノード: Gazebo 内の俯瞰カメラ映像に情報を重ねて mp4 に保存する。

  - 俯瞰カメラ (/overview_camera) 1 フレーム = 動画 1 フレーム
    → シミュレーション時間と動画の時間が一致する (PC が遅くても早送りにならない)
  - 右上: ロボット搭載カメラ (/onboard_camera) のワイプ
  - 左下: オドメトリから描いた軌跡つきミニマップ
  - 左上: シミュレーション時間・タスクの状態
巡回ノードが DONE を出してから 2 秒後に保存して終了する。
"""
import math
import os

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from diffbot_task.arena import ARENA, MARKER_COLORS, OBSTACLES, WAYPOINTS


def to_bgr(msg):
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    img = img[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else img.copy()


def put(img, text, org, scale=0.7, color=(255, 255, 255), thick=2):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thick + 3, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)


class VideoRecorder(Node):
    def __init__(self):
        super().__init__('video_recorder')
        self.declare_parameter('output', os.path.expanduser('~/patrol_demo.mp4'))
        self.declare_parameter('fps', 20.0)  # 俯瞰カメラの update_rate と一致させる
        self.declare_parameter('max_sec', 175.0)
        self.declare_parameter('tail_sec', 2.5)
        self.declare_parameter('controller_label', '')

        self.create_subscription(Image, '/overview_camera', self.on_frame, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(TwistStamped, '/expert_cmd', self.on_expert, 10)
        self.cmd = None
        self.expert = None
        self.create_subscription(Image, '/onboard_camera', self.on_onboard, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(String, '/patrol/status', self.on_status, 10)

        self.writer = None
        self.onboard = None
        self.traj = []
        self.pose = None
        self.status = 'WAITING'
        self.visited = []
        self.t0 = None
        self.n_frames = 0
        self.tail_frames = None
        self.finished = False

    def on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)

    def on_expert(self, msg):
        self.expert = (msg.twist.linear.x, msg.twist.angular.z)

    def on_onboard(self, msg):
        self.onboard = to_bgr(msg)

    def on_odom(self, msg):
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        self.pose = (p.position.x, p.position.y, yaw)
        if not self.traj or math.hypot(self.traj[-1][0] - self.pose[0], self.traj[-1][1] - self.pose[1]) > 0.03:
            self.traj.append(self.pose[:2])

    def on_status(self, msg):
        self.status = msg.data
        if msg.data.startswith('ARRIVED') or msg.data.startswith('DONE'):
            name = msg.data.split()[1] if msg.data.startswith('ARRIVED') else 'H'
            if name not in self.visited:
                self.visited.append(name)

    # ---------- 描画 ----------
    def draw_minimap(self, img):
        scale, pad = 26, 12
        xmin, xmax, ymin, ymax = ARENA
        w, h = int((xmax - xmin) * scale), int((ymax - ymin) * scale)
        ox, oy = 20, img.shape[0] - h - 20

        def px(x, y):  # ワールド座標 → 画像座標 (x 右, y 上)
            return int(ox + (x - xmin) * scale), int(oy + (ymax - y) * scale)

        roi = img[oy - pad:oy + h + pad, ox - pad:ox + w + pad]
        roi[:] = (roi * 0.35).astype(np.uint8)
        cv2.rectangle(img, px(xmin, ymax), px(xmax, ymin), (200, 200, 200), 2)
        for ob in OBSTACLES:
            if ob[0] == 'cyl':
                cv2.circle(img, px(ob[1], ob[2]), int(ob[3] * scale), (200, 120, 90), -1)
            else:
                _, cx, cy, sx, sy, yaw = ob
                pts = []
                for ux, uy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                    lx, ly = ux * sx / 2, uy * sy / 2
                    pts.append(px(cx + lx * math.cos(yaw) - ly * math.sin(yaw),
                                  cy + lx * math.sin(yaw) + ly * math.cos(yaw)))
                cv2.fillPoly(img, [np.array(pts)], (60, 110, 170))
        for name, x, y in WAYPOINTS:
            c = MARKER_COLORS[name]
            cv2.circle(img, px(x, y), 9, c, -1 if name in self.visited else 2)
            put(img, name, (px(x, y)[0] + 10, px(x, y)[1] - 8), 0.5, c, 1)
        if len(self.traj) > 1:
            cv2.polylines(img, [np.array([px(x, y) for x, y in self.traj])], False, (0, 220, 255), 2)
        if self.pose:
            x, y, yaw = self.pose
            tip = px(x + 0.45 * math.cos(yaw), y + 0.45 * math.sin(yaw))
            cv2.circle(img, px(x, y), 6, (0, 128, 255), -1)
            cv2.line(img, px(x, y), tip, (0, 128, 255), 2)
        put(img, 'Map (odometry)', (ox, oy - 18), 0.55, (230, 230, 230), 1)

    def draw_panel(self, img, t):
        cv2.rectangle(img, (0, 0), (img.shape[1], 40), (30, 30, 30), -1)
        put(img, 'ROS 2 Humble + Gazebo Fortress  |  Autonomous patrol learned by imitation learning (LiDAR)',
            (12, 27), 0.62, (255, 255, 255), 1)
        put(img, f'Sim time {t:6.1f} s', (16, 78), 0.8)
        label = self.get_parameter('controller_label').value
        if label:
            put(img, label, (16, img.shape[0] - 290), 0.62, (255, 220, 120), 1)
        if self.cmd is not None and self.expert is not None and self.status.split()[0] == 'AI':
            put(img, f'AI command    v {self.cmd[0]:.2f} m/s  w {self.cmd[1]:+.2f} rad/s',
                (16, 180), 0.6, (255, 220, 120), 1)
            put(img, f'(expert would v {self.expert[0]:.2f} m/s  w {self.expert[1]:+.2f} rad/s)',
                (16, 206), 0.55, (200, 200, 200), 1)
        s = self.status.split()
        if s and s[0] == 'AI':
            put(img, f'AI policy driving -> {s[1]}   ({s[2]} m)', (16, 112), 0.75, (120, 255, 120))
        elif s and s[0] in ('GO', 'AVOID'):
            color = (0, 200, 255) if s[0] == 'AVOID' else (120, 255, 120)
            label = 'Avoiding obstacle' if s[0] == 'AVOID' else 'Moving'
            put(img, f'{label} -> {s[1]}   ({s[2]} m)', (16, 112), 0.75, color)
        elif s and s[0] == 'ARRIVED':
            put(img, f'Arrived at {s[1]}!', (16, 112), 0.8, (120, 255, 255))
        elif s and s[0] == 'DONE':
            put(img, f'Mission complete: {s[2]}, {s[4]}', (16, 112), 0.8, (120, 255, 255))
        else:
            put(img, 'Waiting for start...', (16, 112), 0.75)
        route = '  '.join(f'[{"x" if n in self.visited else " "}] {n}' for n, _, _ in WAYPOINTS)
        put(img, 'Route: ' + route, (16, 146), 0.65)

    def draw_onboard(self, img):
        if self.onboard is None:
            return
        h, w = self.onboard.shape[:2]
        x0, y0 = img.shape[1] - w - 16, 56
        img[y0:y0 + h, x0:x0 + w] = self.onboard
        cv2.rectangle(img, (x0 - 2, y0 - 2), (x0 + w + 1, y0 + h + 1), (255, 255, 255), 2)
        put(img, 'Onboard camera', (x0, y0 + h + 22), 0.55, (255, 255, 255), 1)

    # ---------- フレーム処理 ----------
    def on_frame(self, msg):
        if self.finished:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
        img = to_bgr(msg)
        if self.writer is None:
            out = self.get_parameter('output').value
            os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.writer = cv2.VideoWriter(out, fourcc, self.get_parameter('fps').value,
                                          (img.shape[1], img.shape[0]))
            self.get_logger().info(f'録画開始: {out} ({img.shape[1]}x{img.shape[0]})')
        self.draw_panel(img, t - self.t0)
        self.draw_onboard(img)
        self.draw_minimap(img)
        self.writer.write(img)
        self.n_frames += 1
        if self.n_frames % 250 == 0:
            self.get_logger().info(f'{self.n_frames} frames ({t - self.t0:.0f}s sim)')

        if self.status.startswith('DONE') and self.tail_frames is None:
            self.tail_frames = int(self.get_parameter('tail_sec').value * self.get_parameter('fps').value)
        if self.tail_frames is not None:
            self.tail_frames -= 1
        if (self.tail_frames is not None and self.tail_frames <= 0) or \
                t - self.t0 > self.get_parameter('max_sec').value:
            self.finish()

    def finish(self):
        if self.finished:
            return
        self.finished = True
        if self.writer:
            self.writer.release()
        self.get_logger().info(f'録画終了: {self.n_frames} frames → {self.get_parameter("output").value}')


def main():
    rclpy.init()
    node = VideoRecorder()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
