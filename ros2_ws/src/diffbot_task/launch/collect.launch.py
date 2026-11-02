"""模倣学習用のデモデータ収集 (第5回講義の「テレオペ + ros2 bag record」に相当)。

お手本 (ポテンシャル法) がランダムな目標を回る様子を rosbag2 に記録する。
人がキー操作する代わりにお手本の制御器を「操作者」とし、1 回の起動 = 1 bag、
bag の中の目標 1 つぶん = 1 エピソード として扱う。

  ros2 launch diffbot_task collect.launch.py bag:=~/dataset/rosbags/run0 seed:=0 noise:=true
  ros2 launch diffbot_task collect.launch.py bag:=~/dataset/rosbags/dagger0 seed:=10
      controller:=policy noise:=false goal_timeout:=30     # DAgger (方策が走り、お手本がラベル付け)

収集中は録画用カメラを外した軽いワールドを使う (描画負荷を下げるため)。
巡回が終わると巡回ノードが終了し、それをきっかけに記録も止まる。
"""
import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TOPICS = ['/scan', '/odom', '/cmd_vel', '/expert_cmd', '/patrol/goal', '/patrol/episode', '/patrol/status']


def light_world(src):
    """俯瞰カメラとロボット搭載カメラを取り除いたワールドを一時ファイルに書き出す"""
    sdf = open(src, encoding='utf-8').read()
    sdf = re.sub(r'<model name="overview_camera">.*?</model>', '', sdf, flags=re.S)
    sdf = re.sub(r'<sensor name="onboard_cam".*?</sensor>', '', sdf, flags=re.S)
    path = os.path.join(tempfile.gettempdir(), 'patrol_world_collect.sdf')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(sdf)
    return path


def setup(context):
    share = get_package_share_directory('diffbot_task')
    world = light_world(os.path.join(share, 'worlds', 'patrol_world.sdf'))
    bridge_cfg = os.path.join(share, 'config', 'bridge.yaml')
    bag = os.path.expanduser(LaunchConfiguration('bag').perform(context))
    lc = lambda k: LaunchConfiguration(k).perform(context)

    env = {'LIBGL_ALWAYS_SOFTWARE': '1'}
    gazebo = ExecuteProcess(cmd=['ign', 'gazebo', '-s', '-r', '-v', '1', '--render-engine', 'ogre', world],
                            output='screen', additional_env=env)
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='bridge',
                  parameters=[{'config_file': bridge_cfg, 'use_sim_time': True}], output='log')
    patrol = Node(
        package='diffbot_task', executable='patrol_node', name='patrol_node', output='screen',
        parameters=[{'use_sim_time': True, 'controller': lc('controller'),
                     'model_path': os.path.expanduser(lc('model_path')), 'random_goals': True,
                     'n_random_goals': int(lc('n_goals')), 'seed': int(lc('seed')),
                     'noise': lc('noise').lower() == 'true', 'exit_on_done': True,
                     'pause_sec': 0.5, 'goal_timeout': float(lc('goal_timeout'))}])
    os.makedirs(os.path.dirname(bag) or '.', exist_ok=True)
    record = ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', bag] + TOPICS, output='log')

    return [
        gazebo, bridge, patrol, record,
        RegisterEventHandler(OnProcessExit(target_action=patrol,
                                           on_exit=[EmitEvent(event=Shutdown(reason='collection finished'))])),
        RegisterEventHandler(OnProcessExit(target_action=gazebo,
                                           on_exit=[EmitEvent(event=Shutdown(reason='gazebo exited'))])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag', default_value='~/dataset/rosbags/run0'),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('n_goals', default_value='12'),
        DeclareLaunchArgument('noise', default_value='true'),
        # DAgger: 学習済み方策に走らせ、方策が実際に訪れた状態でお手本のラベル (/expert_cmd) を記録する
        DeclareLaunchArgument('controller', default_value='expert'),
        DeclareLaunchArgument('model_path', default_value='~/checkpoints/patrol_bc/policy.npz'),
        DeclareLaunchArgument('goal_timeout', default_value='45.0'),
        OpaqueFunction(function=setup),
    ])
