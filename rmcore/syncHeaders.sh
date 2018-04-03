#!/bin/bash

piHostName=huww-pi

rsync -rtvz --delete $piHostName:/opt/ros/kinetic/include/ /mnt/c/Source/linux_includes/ros/include/
rsync -rtvz --delete $piHostName:/usr/include/suitesparse/ /mnt/c/Source/linux_includes/suitesparse/
rsync -rtvz --delete $piHostName:/usr/include/pcl-1.7/ /mnt/c/Source/linux_includes/pcl-1.7/

# rsync -rtvz --delete $piHostName:~/catkin_ws/devel/include/rmcore/ /mnt/c/Source/linux_includes/ros/devel/include/rmcore/
