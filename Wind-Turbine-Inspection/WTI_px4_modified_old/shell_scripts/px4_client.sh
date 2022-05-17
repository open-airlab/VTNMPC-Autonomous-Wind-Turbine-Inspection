#!/bin/bash
##Run PX4 firmware

model_name=$1
if [ "$#" != 1 ]; then
    echo -e "usage: source px4_client.sh <model_name>\n"
    return 1
fi
cd ..
no_sim=1 make posix_sitl_lpe gazebo_${model_name}
