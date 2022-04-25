#!/bin/bash

dir=~/rosbag/airsim/car_1cam_lidar32

for abs_path in $dir/* ; do
    path=${abs_path##*/}
    name=${path%.*}
    roslaunch rosbag_to_pcd rosbag_to_pcd.launch bagname:=$dir bagname:=$name
done