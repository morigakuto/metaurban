#!/bin/bash
# =============================================================================
# MetaUrban + ROS2  ワンコマンド起動スクリプト
#
# 前提条件 (Ubuntu PC):
#   1. Docker          : https://docs.docker.com/engine/install/ubuntu/
#   2. NVIDIA Driver   : nvidia-smi で確認
#   3. NVIDIA Container Toolkit:
#        sudo apt-get install -y nvidia-container-toolkit
#        sudo systemctl restart docker
#
# 使い方:
#   ./run.sh build     # イメージをビルド
#   ./run.sh start     # コンテナを起動してシェルに入る
#   ./run.sh demo      # MetaUrban デモを起動
#   ./run.sh ros       # ROS2 ブリッジ + MetaUrban を起動
# =============================================================================
set -e

IMAGE_NAME="metaurban-ros2:latest"

case "${1:-help}" in

  build)
    echo "==> Building Docker image..."
    docker compose build
    echo "==> Done! Run: ./run.sh start"
    ;;

  start)
    echo "==> Starting container..."
    xhost +local:docker 2>/dev/null || true
    docker compose run --rm metaurban bash
    ;;

  demo)
    echo "==> Running MetaUrban demo..."
    xhost +local:docker 2>/dev/null || true
    docker compose run --rm metaurban \
      python3 -m metaurban.examples.drive_in_static_env --density_obj 0.4
    ;;

  ros)
    echo "==> Starting MetaUrban + ROS2 bridge..."
    echo "    This opens a shell. Run these in separate terminals (tmux/screen):"
    echo ""
    echo "    # Terminal 1: ROS2 bridge"
    echo "    ros2 launch metaurban_example_bridge metaurban_example_bridge.launch.py"
    echo ""
    echo "    # Terminal 2: MetaUrban simulator"
    echo "    cd /metaurban/bridges/ros_bridge && python3 ros_socket_server.py"
    echo ""
    xhost +local:docker 2>/dev/null || true
    docker compose run --rm metaurban bash
    ;;

  *)
    echo "Usage: ./run.sh {build|start|demo|ros}"
    echo ""
    echo "  build   Build the Docker image"
    echo "  start   Start container shell"
    echo "  demo    Run MetaUrban demo (static env)"
    echo "  ros     Start container for ROS2 + MetaUrban"
    ;;
esac
