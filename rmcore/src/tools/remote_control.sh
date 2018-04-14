#!/bin/bash

while :
do
    read
    rosservice call /driver_node/remoteController/run "data: true"

    read
    rosservice call /driver_node/remoteController/run "data: false"
done
