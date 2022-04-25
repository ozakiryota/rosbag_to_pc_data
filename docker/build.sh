#!/bin/bash

image="rosbag_to_pc_files"
tag="latest"

docker build . \
    -t $image:$tag \
    --build-arg CACHEBUST=$(date +%s)