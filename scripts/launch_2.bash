#!/bin/bash

echo "--- FlexBE ---"

# --- 環境設定 (一度だけ実行) ---
if [ -f "/home/dockeruser/ros2_ws/install/setup.bash" ]; then
    source /home/dockeruser/ros2_ws/install/setup.bash
fi
export XDG_CONFIG_HOME=/tmp/.chromium
export XDG_CACHE_HOME=/tmp/.chromium

echo -e "\e[32mready\e[0m\n"


# --- 高度な終了処理 ---
LAUNCH_PID=""

cleanup() {
    echo -e "\n\e[33m Detect Ctrl+C...\e[0m"
    if [ -n "$LAUNCH_PID" ]; then
        # Step 1: まずはSIGINTで、丁寧にお願いする
        echo -n "  SIGINT..."
        echo " id:${LAUNCH_PID}"
        kill -SIGINT -- -"$LAUNCH_PID" 2>/dev/null
        
        # Step 2: 2秒待って、まだ生きてるか確認
        sleep 2
        # `ps`コマンドで、まだプロセスが存在するかどうかをチェック
        if ps -p "$LAUNCH_PID" > /dev/null; then
            # Step 3: まだ生きてたら、SIGKILLで強制的に終了させる
            echo -n "  > SIGKILL..."
            echo  "process id: "$LAUNCH_PID
            kill -SIGKILL -- -"$LAUNCH_PID" 2>/dev/null
        else
            echo "  > all processes end"
        fi
    fi
}

trap cleanup INT


# --- メインの起動ループ ---
while true; do
    read -p $'\e[32mPress Enter to launch FlexBE (type \'end\' to quit):\e[0m ' input
    if [[ "$input" == "end" ]]; then
        break
    fi

    echo "Launching FlexBE..."
    setsid ros2 launch flexbe_app flexbe_full.launch.py &
    LAUNCH_PID=$!

    wait "$LAUNCH_PID"
    LAUNCH_PID=""

    echo -e "\e[33mEnded FlexBE\e[0m\n"
done


# --- スクリプトの完全終了処理 ---
trap - INT
echo "Finsish"
