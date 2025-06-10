#!/bin/bash

# ROS2 Humble 直接インストールスクリプト
set -e

echo "=== ROS2 Humble Installation Started ==="

# システムの更新
echo "Updating system packages..."
sudo apt update

# 必要なパッケージのインストール
echo "Installing required packages..."
sudo apt install -y \
    software-properties-common \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    ca-certificates \
    git \
    vim \
    nano \
    build-essential \
    python3-pip \
    python3-dev \
    python3-setuptools

# ROS2リポジトリの追加
echo "Adding ROS2 repository..."
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# パッケージリストの更新
echo "Updating package lists..."
sudo apt update

# ROS2 Humbleのインストール
echo "Installing ROS2 Humble..."
sudo apt install -y \
    ros-humble-desktop-full \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-argcomplete

# rosdepの初期化
echo "Initializing rosdep..."
sudo rosdep init || echo "rosdep already initialized"
rosdep update

# ワークスペースの作成
echo "Creating ROS2 workspace..."
cd ~
mkdir -p ros2_ws/src
cd ros2_ws

# 環境設定の追加
echo "Setting up environment..."
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

if ! grep -q "source ~/ros2_ws/install/setup.bash" ~/.bashrc; then
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
fi

# 初回ビルド
echo "Building workspace..."
colcon build

# 環境変数の設定
echo "Setting environment variables..."
if ! grep -q "export ROS_DISTRO=humble" ~/.bashrc; then
    echo "export ROS_DISTRO=humble" >> ~/.bashrc
fi

if ! grep -q "export ROS_VERSION=2" ~/.bashrc; then
    echo "export ROS_VERSION=2" >> ~/.bashrc
fi

if ! grep -q "export ROS_PYTHON_VERSION=3" ~/.bashrc; then
    echo "export ROS_PYTHON_VERSION=3" >> ~/.bashrc
fi

echo "=== Installation Complete ==="
echo ""
echo "To start using ROS2, run:"
echo "  source ~/.bashrc"
echo "  cd ~/ros2_ws"
echo ""
echo "Test installation with:"
echo "  ros2 --help"
echo "  ros2 topic list"