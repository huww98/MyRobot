#!/bin/bash

piHostName=pi

rsync -rtvz --delete $piHostName:/opt/ros/kinetic/include/ /mnt/c/Source/linux_includes/ros/include/
rsync -rtvz --delete $piHostName:~/catkin_ws/devel/include/my_robot/ /mnt/c/Source/linux_includes/ros/devel/include/my_robot/
