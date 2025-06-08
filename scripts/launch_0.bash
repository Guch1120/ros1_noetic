#!/bin/bash
echo "launch rviz2"
# ROS2 Humbleの環境設定を読み込む
source /opt/ros/humble/setup.bash
if [ -f "/opt/ros2_ws/install/setup.bash" ]; then
    source /opt/ros2_ws/install/setup.bash
fi


# Ctrl+C (SIGINT) を無視する
trap '' INT

while true; do
    # プロンプトメッセージに色を付ける (緑色)
    # \e[32m で緑色開始, \e[0m で色リセット
    read -p $'\e[32mPress Enter to launch RViz2 (or type \'end\' to quit):\e[0m ' input
    if [[ "$input" == "end" ]]; then
        break
    fi
    rviz2
done

# SIGINTのトラップをデフォルトに戻す (念のため)
trap - INT
exec bash # RViz2終了後、またはEnterを押さずにCtrl+Cなどで抜けた場合にシェルを起動
