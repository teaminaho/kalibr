#!/bin/bash

function run_with_docker() {
     \
    sudo docker run -it --rm -v "/:/host" -w "/host/$(pwd)" kalibr:latest "$@"
}

function kalibr_calibrate_cameras() {
    run_with_docker kalibr_calibrate_cameras "$@"
}

function rosbag() {
    run_with_docker rosbag "$@"
}

function kalibr_bagcreater() {
    run_with_docker kalibr_bagcreater "$@"
}
