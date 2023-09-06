#!/bin/bash
docker run --gpus all -it --rm --net host --ipc host --privileged\
    -e DISPLAY=$DISPLAY \
    -v /home/nagata/sources/docker/kinect/:/root/catkin_ws/src/kinect/\
    kinectv2:noetic bash