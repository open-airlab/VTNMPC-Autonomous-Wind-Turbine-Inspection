#!/bin/bash

## Launch Gazebo client

if [ "$#" != 1 ]; then
    echo -e "usage: source gazebo_client.sh <model_name>\n"
    return 1
fi

model_name=$1

#cd ..
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/posix_sitl_lpe
roslaunch gazebo_ros empty_world.launch world_name:=$(pwd)/Tools/sitl_gazebo/worlds/${model_name}.world
