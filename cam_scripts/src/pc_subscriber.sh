#!/bin/bash
for i in $(seq 1 50)
do
    rosrun cam_scripts pointcloud_subscriber.py 1 $i &
done

