#!/bin/bash

CONTAINER_NAME="ros1-noetic-gui-container"

# === 1. ホストのDISPLAYとHOSTNAMEを取得 ===
export DISPLAY=${DISPLAY:-:0}
export ROS_MASTER_URI=http://$(hostname):11311
export ROS_HOSTNAME=$(hostname)

# === 2. コンテナ存在チェック ===
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "コンテナが存在します。起動してアタッチします。"
    docker start -i ${CONTAINER_NAME}
else
    echo "コンテナが存在しません。新しく作成して起動します。"
    docker run -it \
	--gpus all \
        --net=host \
        --env="DISPLAY=${DISPLAY}" \
        --env="QT_X11_NO_MITSHM=1" \
        --env="ROS_MASTER_URI=${ROS_MASTER_URI}" \
        --env="ROS_HOSTNAME=${ROS_HOSTNAME}" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --privileged \
        --name ${CONTAINER_NAME} \
        ros1-noetic-gui
fi
