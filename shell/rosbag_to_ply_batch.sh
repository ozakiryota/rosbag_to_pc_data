#!/bin/bash

dir=~/rosbag/airsim/car_1cam_lidar32

for abs_path in $dir/* ; do
    path=${abs_path##*/}
    name=${path%.*}
    roslaunch rosbag_to_pc_files rosbag_to_ply.launch bagdir:=$dir bagname:=$name
done