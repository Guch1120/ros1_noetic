#!/bin/bash

# === 1. ホストのDISPLAYとHOSTNAMEを取得 ===
export DISPLAY=${DISPLAY:-:0}
export ROS_MASTER_URI=http://$(hostname):11311
export ROS_HOSTNAME=$(hostname)

# === 2. Docker起動 ===
docker run -it \
    --net=host \
    --env="DISPLAY=${DISPLAY}" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="ROS_MASTER_URI=${ROS_MASTER_URI}" \
    --env="ROS_HOSTNAME=${ROS_HOSTNAME}" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --privileged \
    ros1-noetic-gui
