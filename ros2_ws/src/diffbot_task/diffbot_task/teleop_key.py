"""キーボード操作ノード (参考記事の teleop_node を Python で再実装)。

  w / s : 前進 / 後退      a / d : 左旋回 / 右旋回
  space : 停止              q     : 終了
記事では外部パッケージ key_event を使っていたが、ここでは termios で
標準入力を 1 文字ずつ読むので追加パッケージは不要。
"""
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

KEYMAP = {
    'w': (0.3, 0.0),
    's': (-0.3, 0.0),
    'a': (0.0, 1.0),
    'd': (0.0, -1.0),
    ' ': (0.0, 0.0),
}


class TeleopKey(Node):
    def __init__(self):
        super().__init__('teleop_key')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def send(self, lin, ang):
        msg = Twist()
        msg.linear.x, msg.angular.z = lin, ang
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = TeleopKey()
    settings = termios.tcgetattr(sys.stdin)
    print(__doc__)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                if key == 'q':
                    break
                if key in KEYMAP:
                    node.send(*KEYMAP[key])
                    print(f'key={key!r} -> linear={KEYMAP[key][0]:+.1f} angular={KEYMAP[key][1]:+.1f}')
            rclpy.spin_once(node, timeout_sec=0)
    finally:
        node.send(0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
