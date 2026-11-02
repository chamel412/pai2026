"""巡回デモ: Gazebo + ブリッジ + 巡回ノード (+ 録画ノード) をまとめて起動する。

  ros2 launch diffbot_task patrol.launch.py controller:=policy model_path:=~/checkpoints/patrol_bc/policy.npz
  ros2 launch diffbot_task patrol.launch.py controller:=expert record:=false     # お手本で評価だけ
  ros2 launch diffbot_task patrol.launch.py gui:=true                            # Gazebo の GUI も表示
録画ありのときは録画終了で、録画なしのときは巡回完了で launch 全体が自動で終了する。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = get_package_share_directory('diffbot_task')
    world = os.path.join(share, 'worlds', 'patrol_world.sdf')
    bridge_cfg = os.path.join(share, 'config', 'bridge.yaml')
    lc = lambda k: LaunchConfiguration(k).perform(context)
    gui = lc('gui').lower() == 'true'
    record = lc('record').lower() == 'true'
    controller = lc('controller')
    model_path = os.path.expanduser(lc('model_path'))

    gz_cmd = ['ign', 'gazebo', '-r', '-v', '2', '--render-engine', lc('render_engine'), world]
    if not gui:
        gz_cmd.insert(2, '-s')  # サーバのみ (センサの描画は行う)
    # WSL では GPU(d3d12) 経由の OpenGL で描画が落ちやすいので、既定はソフトウェア描画
    env = {'LIBGL_ALWAYS_SOFTWARE': '1'} if lc('sw_render').lower() == 'true' else {}
    gazebo = ExecuteProcess(cmd=gz_cmd, output='screen', additional_env=env)

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='bridge',
        parameters=[{'config_file': bridge_cfg, 'use_sim_time': True}], output='log')

    patrol = Node(
        package='diffbot_task', executable='patrol_node', name='patrol_node',
        parameters=[{'use_sim_time': True, 'controller': controller, 'model_path': model_path,
                     'exit_on_done': not record}],
        output='screen')

    actions = [gazebo, bridge, patrol,
               # Gazebo が異常終了したら、他のノードを待たせ続けず launch 全体を終了する
               RegisterEventHandler(OnProcessExit(
                   target_action=gazebo, on_exit=[EmitEvent(event=Shutdown(reason='gazebo exited'))]))]
    if record:
        label = ('Controller: AI policy (MLP trained by imitation learning)' if controller == 'policy'
                 else 'Controller: hand-designed potential field (expert)')
        recorder = Node(
            package='diffbot_task', executable='video_recorder', name='video_recorder',
            parameters=[{'use_sim_time': True, 'output': lc('output'), 'controller_label': label}],
            output='screen')
        actions.append(recorder)
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=recorder, on_exit=[EmitEvent(event=Shutdown(reason='recording finished'))])))
    else:
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=patrol, on_exit=[EmitEvent(event=Shutdown(reason='patrol finished'))])))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('record', default_value='true'),
        DeclareLaunchArgument('output', default_value=os.path.expanduser('~/patrol_demo.mp4')),
        DeclareLaunchArgument('controller', default_value='expert'),   # expert / policy
        DeclareLaunchArgument('model_path', default_value='~/checkpoints/patrol_bc/policy.npz'),
        DeclareLaunchArgument('render_engine', default_value='ogre'),  # WSL では ogre2 が落ちるため
        DeclareLaunchArgument('sw_render', default_value='true'),
        OpaqueFunction(function=setup),
    ])
