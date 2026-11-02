from glob import glob
from setuptools import setup

package_name = 'diffbot_task'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='Imitation-learned autonomous patrol for a differential drive robot (ROS 2 + Gazebo)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'patrol_node = diffbot_task.patrol_node:main',
            'teleop_key = diffbot_task.teleop_key:main',
            'video_recorder = diffbot_task.video_recorder:main',
            'rosbag2dataset = diffbot_task.rosbag2dataset:main',
            'train_bc = diffbot_task.train_bc:main',
            'debug_policy = diffbot_task.debug_policy:main',
        ],
    },
)
