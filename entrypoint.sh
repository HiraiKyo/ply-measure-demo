#!/bin/bash
source /opt/ros/noetic/setup.bash
cd /root/catkin_ws
catkin build
echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

# For React
cd /root/catkin_ws/src/ply-measure-demo/web
npm i

# 最後に終了しないコマンドを実行
exec tail -f /dev/null