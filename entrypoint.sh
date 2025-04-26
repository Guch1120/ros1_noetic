#!/bin/bash
source /opt/ros/noetic/setup.bash

terminator --nwe-tab --command "roscore" --layout=default &
terminator --nwe-tab --command "bash" --layout=default &
terminator --nwe-tab --command "bash" --layout=default &

