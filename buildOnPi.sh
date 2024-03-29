#!/bin/bash

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color

piHostName="huww-pi"

echo syncing code to $piHostName
rsync -rtz --delete ./rmcore/ $piHostName:~/catkin_ws/src/rmcore/
rsync -rtz --delete ./rmvisual/ $piHostName:~/catkin_ws/src/rmvisual/

ssh $piHostName 'CLICOLOR_FORCE=1 catkin_make -j2 --directory ~/catkin_ws -DCMAKE_BUILD_TYPE=Debug --only-pkg-with-deps rmcore -- rmcore_driver_node rmvisual_node' || {
    echo -e "${RED}build failed${NC}"
    exit 1
}

echo -e "${GREEN}build Success${NC}"
