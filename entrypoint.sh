#!/bin/bash
source /opt/ros/noetic/setup.bash

# terminatorは起動しない（まずはシンプルに確認する）
echo "起動しました！"

roscore > /tmp/roscore.log 2>&1 &
sleep 2

for i in {1..5}; do
 if rostopic list > /dev/null 2>&1; then
  echo "roscore "

# 最後に必ずbashを実行
exec bash -l
