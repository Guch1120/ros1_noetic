#!/bin/bash

xhost +local:docker

docker run -it \
  --gpus all \
  --name ros1-noetic-gui-container \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $HOME/.Xauthority:/home/dockeruser/.Xauthority \
  -e XAUTHORITY=/home/dockeruser/.Xauthority \
  --privileged \
  ros1-noetic-gui \
  /entrypoint.sh
