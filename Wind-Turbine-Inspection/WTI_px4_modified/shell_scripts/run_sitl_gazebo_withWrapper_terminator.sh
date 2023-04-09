#!/bin/bash
#
# To run SITL GAZEBO with ros wrapper

if [ "$#" == 0 ]; then
    echo -e "usage: source run_sitl_gazebo_withWrapper.sh <model_name>\n"
    return 1
fi

model_name=$1
additional_name=$2

## Check for plugins Build
curr_dir=$PWD
echo $PWD
cd ../Tools/sitl_gazebo/
if [ ! -d "Build" ]; then
    echo -e "Build folder in sitl_gazebo does not exist. Creating it!";
    mkdir -p Build && cd Build && cmake ..;
else
    cd Build;
fi
make all
cd $curr_dir

## Run PX4 firmware
terminator -e ""
./new_tab.sh "source px4_client.sh ${model_name};"

## Launch Gazebo client
if [ "$#" == 2 ]; then
    ./new_tab.sh "source ./shell_scripts/gazebo_client.sh ${model_name}_${additional_name};"
else
    ./new_tab.sh "source ./shell_scripts/gazebo_client.sh ${model_name};"
fi

## Launch Mavros
#./new_tab.sh "roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557" gcs_url:="udp-b://@""
./new_tab.sh "roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557";"
## Run mocap_gazebo
./new_tab.sh "roslaunch mocap mocap_bridge_gazebo.launch;"


