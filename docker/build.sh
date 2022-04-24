#!/bin/bash

image="rosbag_to_pcd"
tag="latest"

docker build . \
    -t $image:$tag \
    --build-arg CACHEBUST=$(date +%s)