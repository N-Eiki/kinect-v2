#!/bin/bash
docker run --gpus all -it --rm --net host --ipc host --privileged\
    -e DISPLAY=$DISPLAY \
    -v /home/rllab/nagata/kinect-v2/kinect:/root/catkin_ws/src/kinect \
    --name kinectv2-cont\
    kinectv2:hsr_noetic  bash