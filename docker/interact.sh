#!/bin/bash

xhost +

image="rosbag_to_pcd"
tag="latest"

docker run \
	-it \
	--rm \
	-e "DISPLAY" \
	-v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	-v $(pwd)/../save:/root/catkin_ws/src/$image/save \
	-v $(pwd)/../launch:/root/catkin_ws/src/$image/launch \
	-v $(pwd)/../rviz_config:/root/catkin_ws/src/$image/rviz_config \
	-v $HOME/rosbag:/root/rosbag \
	$image:$tag