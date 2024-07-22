#!/bin/bash

if [[ $1 == "-f" ]] ; then
    bash clean.sh
fi

colcon build --packages-up-to scp_message
colcon build

source install/setup.bash
