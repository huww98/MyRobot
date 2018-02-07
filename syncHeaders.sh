#!/bin/bash

rsync -rtvz --delete pi:/opt/ros/kinetic/include/ /mnt/c/Source/linux_includes/ros/include/
