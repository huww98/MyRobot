#!/bin/bash

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color

echo syncing code
rsync -rtz --delete ./rmcore/ ~/catkin_ws/src/rmcore/

source /opt/ros/kinetic/setup.bash

catkin_make --directory ~/catkin_ws -DCMAKE_BUILD_TYPE=Debug --only-pkg-with-deps rmcore || {
    echo -e "${RED}build failed${NC}"
    exit 1
}

echo -e "${GREEN}build Success${NC}"
