#!/bin/bash
# set -e を外す。ros2 launchをCtrl+Cで終了したときにスクリプト全体が終わらないようにするため。

echo "launch flexbe"

# ROS2 Humbleの環境設定を読み込む
source /opt/ros/humble/setup.bash

# ワークスペースの環境を読み込む
if [ -f "/home/dockeruser/ros2_ws/install/setup.bash" ]; then
    source /home/dockeruser/ros2_ws/install/setup.bash
else
    echo "エラー: ros2_ws/install/setup.bash が見つかりません。"
    echo "'bash scripts/setup_flexbe.sh' を先に実行しましたか？"
    exit 1
fi

# nwjs (Chromium) のUIクラッシュを回避するための環境変数を設定
export XDG_CONFIG_HOME=/tmp/.chromium
export XDG_CACHE_HOME=/tmp/.chromium

# --- 起動ループ ---
# Ctrl+C (SIGINT) を無視して、ループが勝手に終わらないようにする
trap '' INT

while true; do
    # プロンプトメッセージ
    read -p $'\e[32mEnterキーを押してFlexBEを起動 (終了するには \'end\' と入力):\e[0m ' input

    # 'end' と入力されたらループを抜ける
    if [[ "$input" == "end" ]]; then
        break
    fi

    # FlexBEを起動
    ros2 launch flexbe_app flexbe_full.launch.py
done

# SIGINTのトラップをデフォルトに戻す
trap - INT
exec bash # FlexBE終了後、またはEnterを押さずにCtrl+Cなどで抜けた場合にシェルを起動
