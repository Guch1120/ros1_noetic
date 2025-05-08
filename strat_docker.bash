#!/bin/bash

CONTAINER_NAME="ros1-noetic-gui-container"

# === 1. ホストのDISPLAYとHOSTNAMEを取得 ===
export DISPLAY=${DISPLAY:-:0}
export ROS_MASTER_URI=http://$(hostname):11311
export ROS_HOSTNAME=$(hostname)

# === 2. コンテナ存在チェック ===
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "コンテナが存在。起動してアタッチ"
    docker start -i ${CONTAINER_NAME}
else
    echo "コンテナが存在しません。新しく作成して起動"
    docker run -it \
	--gpus all \
        --name ${CONTAINER_NAME} \
        --net=host \
        --env="DISPLAY=${DISPLAY}" \
        --env="QT_X11_NO_MITSHM=1" \
        --env="ROS_MASTER_URI=${ROS_MASTER_URI}" \
        --env="ROS_HOSTNAME=${ROS_HOSTNAME}" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume="$(pwd)/scripts:/home/dockeruser/scripts:rw" \
        --volume="$(pwd)/terminator_config:/home/dockeruser/terminator_config:rw" \
        --privileged \
        ros1-noetic-gui
fi
