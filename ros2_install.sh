#!/bin/bash

# プロキシ環境対応ROS2インストールスクリプト
set -e

echo "=== Proxy Environment ROS2 Installation ==="

# 既存のプロキシ設定を保持しつつ、証明書を適用
echo "Configuring proxy with SSL certificates..."
export http_proxy="http://proxy:8080"
export https_proxy="http://proxy:8080"
export HTTP_PROXY="http://proxy:8080"
export HTTPS_PROXY="http://proxy:8080"
export no_proxy="localhost,127.0.0.1,::1"
export NO_PROXY="localhost,127.0.0.1,::1"

# apt プロキシ設定を適用
echo "Configuring apt proxy settings..."
sudo tee /etc/apt/apt.conf.d/95proxies > /dev/null <<EOF
Acquire::http::Proxy "http://proxy:8080";
Acquire::https::Proxy "http://proxy:8080";
Acquire::ftp::Proxy "http://proxy:8080";
Acquire::https::CaInfo "/usr/local/share/ca-certificates/envoy-mitmproxy-ca-cert.crt";
EOF

# DNS設定はそのまま使用（既に適切に設定されている）
echo "Using existing DNS configuration..."

# CA証明書の更新
echo "Updating CA certificates..."
sudo update-ca-certificates

# 問題のあるリポジトリを一時的に移動
echo "Temporarily disabling problematic repositories..."
sudo mkdir -p /tmp/disabled-repos
sudo mv /etc/apt/sources.list.d/llvm* /tmp/disabled-repos/ 2>/dev/null || true

# aptキャッシュをクリア
echo "Cleaning apt cache..."
sudo apt clean
sudo rm -rf /var/lib/apt/lists/*

# 基本的なパッケージ情報を更新
echo "Updating package lists (basic repositories only)..."
sudo apt update

# 必要最小限のパッケージをインストール
echo "Installing essential packages..."
sudo apt install -y \
    curl \
    wget \
    gnupg2 \
    ca-certificates \
    software-properties-common

# ROS2のGPGキーをプロキシ経由でダウンロード
echo "Adding ROS2 GPG key..."
curl --cacert /usr/local/share/ca-certificates/envoy-mitmproxy-ca-cert.crt \
     --proxy http://proxy:8080 \
     -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
     sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

# ROS2リポジトリを追加
echo "Adding ROS2 repository..."
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# パッケージリストを再更新
echo "Updating package lists with ROS2 repository..."
sudo apt update

# ROS2の基本パッケージをインストール
echo "Installing ROS2 Humble base..."
sudo apt install -y \
    ros-humble-ros-base \
    python3-rosdep \
    python3-colcon-common-extensions

# rosdepの初期化
echo "Initializing rosdep..."
sudo rosdep init || echo "rosdep already initialized"
rosdep update

# 追加パッケージ（可能であれば）
echo "Installing additional packages..."
sudo apt install -y \
    ros-humble-demo-nodes-cpp \
    ros-humble-demo-nodes-py \
    python3-argcomplete \
    git \
    build-essential \
    python3-pip \
    || echo "Some packages could not be installed, continuing..."

# ワークスペースの作成
echo "Creating workspace..."
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# 環境設定
echo "Setting up environment..."
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

if ! grep -q "source ~/ros2_ws/install/setup.bash" ~/.bashrc; then
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
fi

# 環境変数
echo "export ROS_DISTRO=humble" >> ~/.bashrc
echo "export ROS_VERSION=2" >> ~/.bashrc

# 初回ビルド
echo "Building workspace..."
colcon build

# DNS設定を復元（この環境では不要）
echo "DNS configuration maintained..."

echo "=== Installation Complete ==="
echo ""
echo "Run the following to start using ROS2:"
echo "  source ~/.bashrc"
echo "  cd ~/ros2_ws"
echo ""
echo "Test with:"
echo "  ros2 --help"
echo "  ros2 pkg list"
