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


if ! pgrep -x "terminator" > /dev/null; then
  # terminatorをバックグラウンドで起動
  dbus-launch terminator &
else
  echo "terminator is already running."
fi
exec "$@"