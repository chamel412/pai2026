#!/bin/bash
# Ubuntu 22.04 (WSL2 可) に ROS 2 Humble + Ignition Gazebo Fortress + 必要ツールを導入する
# 実行: sudo bash install_ros2_humble.sh
set -e
export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get install -y software-properties-common curl gnupg lsb-release locales
locale-gen en_US.UTF-8
add-apt-repository -y universe

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  > /etc/apt/sources.list.d/ros2.list

apt-get update
# ros-humble-ros-gz で Ignition Gazebo Fortress と ros_gz_bridge が入る
apt-get install -y ros-humble-ros-base ros-humble-ros-gz ros-humble-launch-ros \
  python3-colcon-common-extensions python3-opencv python3-numpy ffmpeg xterm

grep -q "/opt/ros/humble/setup.bash" /etc/bash.bashrc || echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc
echo "install done"
