#!/bin/bash

## Bash script for setting up ROS Melodic (with Gazebo 9) development environment for PX4 on Ubuntu LTS (18.04).
## It installs the common dependencies for all targets (including Qt Creator)
##
## Installs:
## - Common dependencies libraries and tools as defined in `ubuntu_sim_common_deps.sh`
## - ROS Melodic (including Gazebo9)
## - MAVROS

# Preventing sudo timeout https://serverfault.com/a/833888
trap "exit" INT TERM; trap "kill 0" EXIT; sudo -v || exit $?; sleep 1; while true; do sleep 60; sudo -nv; done 2>/dev/null &

if [[ $(lsb_release -sc) == *"xenial"* ]]; then
  echo "OS version detected as $(lsb_release -sc) (16.04)."
  echo "ROS Melodic requires at least Ubuntu 18.04."
  echo "Exiting ...."
  return 1;
fi

# Ubuntu Config
echo "We must first remove modemmanager"
sudo apt-get remove modemmanager -y
echo "Add user to dialout group for serial port access (reboot required)"
sudo usermod -a -G dialout $USER

# Common dependencies
echo "Installing common dependencies"
sudo apt-get update -y
sudo apt-get install git zip qtcreator cmake build-essential genromfs ninja-build exiftool astyle -y
# make sure xxd is installed, dedicated xxd package since Ubuntu 18.04 but was squashed into vim-common before
which xxd || sudo apt install xxd -y || sudo apt-get install vim-common --no-install-recommends -y
# Required python packages
sudo apt-get install python-argparse python-empy python-toml python-numpy python-dev python-pip -y
sudo -H pip install --upgrade pip
sudo -H pip install pandas jinja2 pyserial pyyaml
# optional python tools
sudo -H pip install pyulog
# install arm-none-eabi-gcc compiler
sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
sudo apt-get update
sudo apt-get install gcc-arm-embedded

# ROS Melodic
## Gazebo simulator dependencies
sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y

## ROS Gazebo: http://wiki.ros.org/melodic/Installation/Ubuntu
## Setup keys
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
## For keyserver connection problems substitute hkp://pgp.mit.edu:80 or hkp://keyserver.ubuntu.com:80 above.
sudo apt-get update
## Get ROS/Gazebo
sudo apt install ros-melodic-desktop-full -y
## Initialize rosdep
sudo rosdep init
rosdep update

## Setup environment variables
rossource="source /opt/ros/melodic/setup.bash"
if grep -Fxq "$rossource" ~/.bashrc; then echo ROS setup.bash already in .bashrc;
else echo "$rossource" >> ~/.bashrc; fi
eval $rossource
#catkin_ws_source="source ~/catkin_ws/devel/setup.bash"
#if grep -Fxq "$catkin_ws_source" ~/.bashrc; then echo ROS catkin_ws setup.bash already in .bashrc; 
#else echo "$catkin_ws_source" >> ~/.bashrc; fi
#eval $catkin_ws_source
## Install rosinstall and other dependencies
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y

# MAVROS:
## Install dependencies
sudo apt-get install python-catkin-tools python-rosinstall-generator -
sudo apt-get install ros-melodic-mavros ros-melodic-mavros-msgs ros-melodic-mavros-extras

#Install geographiclib
sudo apt install geographiclib-tools -y
echo "Downloading dependent script 'install_geographiclib_datasets.sh'"
# Source the install_geographiclib_datasets.sh script directly from github
install_geo=$(wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh -O -)
wget_return_code=$?
# If there was an error downloading the dependent script, we must warn the user and exit at this point.
if [[ $wget_return_code -ne 0 ]]; then echo "Error downloading 'install_geographiclib_datasets.sh'. Sorry but I cannot proceed further :("; exit 1; fi
# Otherwise source the downloaded script.
sudo bash -c "$install_geo"

# Setting up PX4-firmware
clone_dir = ~/Wind-Turbine-Inspection/WTI_px4_modified
cd $clone_dir/Tools/sitl_gazebo
mkdir Build

## Adding PX4-Firmware to ROS environment variables
gazebo_model_export="export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$clone_dir/Tools/sitl_gazebo/models"
if grep -Fxq "$gazebo_model_export" ~/.bashrc; then echo GAZEBO_MODEL_PATH already in .bashrc;
else echo "$gazebo_model_export" >> ~/.bashrc; fi
eval $gazebo_model_export
gazebo_plugin_export="export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$clone_dir/Tools/sitl_gazebo/Build"
if grep -Fxq "$gazebo_plugin_export" ~/.bashrc; then echo GAZEBO_PLUGIN_PATH already in .bashrc;
else echo "$gazebo_plugin_export" >> ~/.bashrc; fi
eval $gazebo_plugin_export
gazebo_database_export="export GAZEBO_MODEL_DATABASE_URI=$GAZEBO_MODEL_DATABASE_URI:http://get.gazebosim.org"
if grep -Fxq "$gazebo_database_export" ~/.bashrc; then echo GAZEBO_MODEL_DATABASE_URI already in .bashrc;
else echo "$gazebo_database_export" >> ~/.bashrc; fi
eval $gazebo_database_export
gazebo_sitl_export="export SITL_GAZEBO_PATH=$SITL_GAZEBO_PATH:$clone_dir/Tools/sitl_gazebo"
if grep -Fxq "$gazebo_sitl_export" ~/.bashrc; then echo SITL_GAZEBO_PATH already in .bashrc;
else echo "$gazebo_sitl_export" >> ~/.bashrc; fi
eval $gazebo_sitl_export

cd Build
cmake ..
make sdf
make
cd $clone_dir
