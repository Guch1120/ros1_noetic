#!/bin/bash
set -e

# ROSセットアップ
source /opt/ros/noetic/setup.bash

# terminatorの設定ファイルをコピー（初回だけ）
mkdir -p ~/.config/terminator/

if ! cmp -s /home/dockeruser/terminator_config/config ~/.config/terminator/config; then
  cp /home/dockeruser/terminator_config/config ~/.config/terminator/config
  chown dockeruser:dockeruser ~/.config/terminator/config
fi

# roscore起動（バックグラウンド）
roscore &
sleep 2

# terminator起動（もし既に動いてたら起動しない）
if ! pgrep -x "terminator" > /dev/null; then
  # terminator起動（dockeruserとして）
  terminator -m -l default --config /home/dockeruser/terminator_config/config &
else
  echo "terminator is already running."
fi

# bashを起動し続ける
exec bash

