set -e

# ROS2 Humbleの環境設定を読み込む
source /opt/ros/humble/setup.bash

if [ -f "/opt/ros2_ws/install/setup.bash" ]; then
  source /opt/ros2_ws/install/setup.bash
fi

# terminatorの設定ファイルがあればコピーする
if [ -f "/home/dockeruser/terminator_config/config" ]; then
  cp /home/dockeruser/terminator_config/config /home/dockeruser/.config/terminator/config
  chown dockeruser:dockeruser /home/dockeruser/.config/terminator/config
fi

# もし既に動いてたら起動しない
if ! pgrep -x "terminator" > /dev/null; then
  # terminatorをバックグラウンドで起動
  dbus-launch terminator &
else
  echo "terminator is already running."
fi


# 渡されたコマンドを実行する
exec "$@"