#!/bin/bash

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color

piHostName="huww-pi"

echo syncing code to $piHostName
rsync -rt --delete ./ $piHostName:~/catkin_ws/src/my_robot/

ssh $piHostName 'source /opt/ros/kinetic/setup.bash && export CLICOLOR_FORCE=1 && catkin_make --directory ~/catkin_ws --only-pkg-with-deps my_robot' || {
    echo -e "${RED}build failed${NC}"
    exit 1
}

echo -e "${GREEN}build Success${NC}"
