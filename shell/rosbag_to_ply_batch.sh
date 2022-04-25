#!/bin/bash

if [ $# != 1 ]; then
	echo "Usage: ./rosbag_to_ply_batch.sh DIR"
	exit 1
fi

dir=$1

for abs_path in $dir/* ; do
    path=${abs_path##*/}
    name=${path%.*}
    roslaunch rosbag_to_pc_files rosbag_to_ply.launch bagdir:=$dir bagname:=$name
done