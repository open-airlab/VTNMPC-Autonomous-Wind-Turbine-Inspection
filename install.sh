#!/bin/bash

echo "========================================="
echo "Starting Installation process"
echo "========================================="


# downloading github repo
echo "========================================="
echo "downloading github repo"
echo "========================================="
cd
git clone git@github.com:abdelhakim96/WT_hakim_test.git





# Download mavros dependecies

echo "========================================="
echo "Downloading mavros dependecies"
echo "========================================="
sudo apt-get install ros-melodic-mavros*
sudo apt-get install xdotool
sudo apt-get ros-mavros-mav-msgs 





# copy PX4 folder to home directory
echo "========================================="
echo "copying PX4 folder to home"
echo "========================================="
cd
cd WT_hakim_test
cp -R Wind-Turbine-Inspection ~/

# copy catkin files folder to catkin workspace
echo "========================================="
echo "copying catkin files to catkin_ws"
echo "========================================="
cd
cd WT_hakim_test
cp -R WTI_catkin ~/catkin_ws/src


# remove NMPC code generator folder to be able to use catkin build
echo "========================================="
echo "removing code generation files"
echo "========================================="
cd ~/catkin_ws/src/WTI_catkin
rm -r NMPC_code_generator
rm -r wind_estimator




# build catkin ws
echo "========================================="
echo "building ws"
echo "========================================="
cd ~/catkin_ws/
catkin clean
catkin build



# install_dependencies_and_setup_px4


echo "========================================="
echo "installing dependencies and setting up px4"
echo "========================================="
cd
cd Wind-Turbine-Inspection
./install_dependencies_and_setup_px4_modified.sh

#Add alias for arming the drone and setting the mode to offboard.


echo "========================================="
echo "Adding alias for arming the drone and setting the mode to offboard.
"
echo "========================================="

#echo 'alias arm='rosrun mavros mavsafety arm'' >> ~/.bashrc 
#echo 'alias disarm='rosrun mavros mavsafety disarm'' >> ~/.bashrc 
#echo 'alias offboard='rosrun mavros mavsys mode -c OFFBOARD'' >> ~/.bashrc 




