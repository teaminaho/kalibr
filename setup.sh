#!/bin/bash

function run_with_docker() {
    docker run --rm -v "/:/host" -w "/host/$(pwd)" kalibr:latest "$@"
}

function kalibr_calibrate_cameras() {
    run_with_docker kalibr_calibrate_cameras --dont-show-report "$@"
}

function rosbag() {
    run_with_docker rosbag "$@"
}

function kalibr_bagcreater() {
    run_with_docker kalibr_bagcreater "$@"
}

source /opt/ros/kinetic/setup.bash
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
echo "source /home/catkin_ws/devel/setup.bash" >> ~/.bashrc

cd /home/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release -Wno-dev
