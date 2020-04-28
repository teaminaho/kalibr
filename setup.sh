#!/bin/bash

source /opt/ros/kinetic/setup.bash
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
echo "source /home/catkin_ws/devel/setup.bash" >> ~/.bashrc

cd /home/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release -Wno-dev
