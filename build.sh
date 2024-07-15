#!/bin/bash

if [[ $1 == "-f" ]] ; then
    bash clean.sh
fi

colcon build

source install/setup.bash
