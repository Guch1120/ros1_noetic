#!/bin/bash

CONTAINER_NAME="ros1-noetic-gui-container"

# ホストのDISPLAYとHOSTNAMEを取得
export DISPLAY=${DISPLAY:-:0}
export ROS_MASTER_URI=http://$(hostname):11311
export ROS_HOSTNAME=$(hostname)

# コンテナが存在するかチェック
if [ "$(docker ps -aq -f name=${CONTAINER_NAME})" ]; then
    if [ "$(docker ps -q -f name=${CONTAINER_NAME})" ]; then
        echo "コンテナはすでに起動中です。アタッチします。"
    else
        echo "コンテナが存在します。起動してアタッチします。"
        docker start ${CONTAINER_NAME}
    fi
else
    echo "コンテナが存在しません。新しく作成します。"
    docker run -itd \
        --name ${CONTAINER_NAME} \
        --net=host \
        --env="DISPLAY=${DISPLAY}" \
        --env="QT_X11_NO_MITSHM=1" \
        --env="ROS_MASTER_URI=${ROS_MASTER_URI}" \
        --env="ROS_HOSTNAME=${ROS_HOSTNAME}" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --privileged \
        ros1-noetic-gui
fi

# ★ここでコンテナに入る！！
docker exec -it ${CONTAINER_NAME} bash
