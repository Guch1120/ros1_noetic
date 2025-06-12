#!/bin/bash
set -e

# ROS環境を読み込む
source /opt/ros/humble/setup.bash
if [ -f "/home/dockeruser/ros2_ws/install/setup.bash" ]; then
  source /home/dockeruser/ros2_ws/install/setup.bash
fi

# terminatorの設定ファイルがあればコピーする
if [ -f "/home/dockeruser/terminator_config/config" ]; then
  mkdir -p /home/dockeruser/.config/terminator
  cp /home/dockeruser/terminator_config/config /home/dockeruser/.config/terminator/config
  chown dockeruser:dockeruser /home/dockeruser/.config/terminator/config
fi

# dbus-run-session を使って、クリーンなD-Busセッションを開始し、
# その中でterminatorと、コンテナのメインコマンドを実行する
exec dbus-run-session -- bash -c '
  set -e
  terminator &
  sleep 1
  exec "$@"
'