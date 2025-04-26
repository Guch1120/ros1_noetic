#!/bin/bash
source /opt/ros/noetic/setup.bash
roscore &
sleep 2
exec "$@"
