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
