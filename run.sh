#!/bin/bash

# Allow local X11 connections for RViz
xhost +local:root > /dev/null 2>&1

# Run the docker container
docker run -it --rm \
    --privileged \
    --ipc=host \
    -v /dev:/dev \
    -v /run/udev:/run/udev:ro \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e QT_X11_NO_MITSHM=1 \
    -v $(pwd):/run_config \
    --network host \
    openvins_realsense_humble bash -c "ln -sf /run_config/src/* /root/ros2_ws/src/ && exec bash"
