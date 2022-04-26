#!/bin/bash

image="rosbag_to_pc_data"
tag="latest"

docker build . \
    -t $image:$tag \
    --build-arg CACHEBUST=$(date +%s)