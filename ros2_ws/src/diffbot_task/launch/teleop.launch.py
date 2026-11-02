"""手動操作モード (参考記事の nav_teleop.launch.py に相当)。

Gazebo GUI + ブリッジ + キーボード操作ノード(xterm 内) を起動する。
  ros2 launch diffbot_task teleop.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('diffbot_task')
    world = os.path.join(share, 'worlds', 'patrol_world.sdf')
    bridge_cfg = os.path.join(share, 'config', 'bridge.yaml')

    return LaunchDescription([
        # WSL2 では ogre2 の描画が落ちるため ogre + ソフトウェア描画にする
        ExecuteProcess(cmd=['ign', 'gazebo', '-r', '--render-engine', 'ogre', world], output='screen',
                       additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'}),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='bridge',
             parameters=[{'config_file': bridge_cfg, 'use_sim_time': True}], output='screen'),
        # キー入力とログが混ざらないよう別ウィンドウ(xterm)で起動
        Node(package='diffbot_task', executable='teleop_key', name='teleop_key',
             prefix='xterm -e', output='screen'),
    ])
