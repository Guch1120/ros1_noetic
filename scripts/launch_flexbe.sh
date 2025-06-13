#!/bin/bash
set -e

echo "--- [Auto-Launch] FlexBEを起動します ---"

# ワークスペースの環境を読み込む
# このスクリプトはコンテナ起動時にTerminatorから直接呼ばれるので
# フルパスで指定するのが安全
if [ -f "/home/dockeruser/ros2_ws/install/setup.bash" ]; then
    source /home/dockeruser/ros2_ws/install/setup.bash
fi

# クラッシュ回避用の環境変数を設定
export XDG_CONFIG_HOME=/tmp/.chromium
export XDG_CACHE_HOME=/tmp/.chromium

# FlexBEを即座に起動
ros2 launch flexbe_app flexbe_full.launch.py
