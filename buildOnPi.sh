#!/bin/bash

piHostName="pi"

echo syncing code to $piHostName
rsync -rt --delete ./ $piHostName:~/catkin_ws/src/my_robot/

ssh $piHostName 'source /opt/ros/kinetic/setup.bash && catkin_make --directory ~/catkin_ws --only-pkg-with-deps my_robot'