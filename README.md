# Autonomous Wind Turbine Inspection Framework Enabled by Visual Tracking Nonlinear Model Predictive Control (VT-NMPC)

<p align=center>
<img width=500 src="vtnmpc.gif" />
</p>








This repository contains the code and simulation files for the submitted paper titled "Visual Tracking Nonlinear Model Predictive Control Method for Autonomous Wind Turbine Inspection". 
[Link to paper](https://ieeexplore.ieee.org/abstract/document/10406329?casa_token=E3nhD10-h10AAAAA:aJ4iZipbMURKRQeuiITx9BC0teooc_D5BAb71Vfi4Cw4mCCRAa4WPwd7oUVGSTe0xXLezf0lSw) 

For this purpose, a time optimal path planner and a Visual tracking MPC is developed. 




We provide a general inspection framework, that takes the dimensions of the wind turbine as input, and provides the optimal attitude rate and thrust command to the drone to acheive optimal coverage. 

![My Image](abstract_vtmpc.png)


The approach is modular, where the global plan for inspecting is provided through a time optimal graph based path planner. The output of the path planner is sequentially input to a NMPC with visual tracking costs, that allows the drone to acheive optimal pose relative to the surface for best heading, incidence angle and distance from the surface. More details on the method can be found soon through the paper submitted for publication.




# Install Ubuntu 18.04 and ROS melodic

# Clone repository
cd
git clone git@github.com:open-airlab/VTNMPC-Autonomous-Wind-Turbine-Inspection.git

# Download PX4 folder and place it inside the Wind-Turbine-Inspection folder

# Copy the WTI_catkin folder inside the catkin_ws

# Download MAVROS dependencies
sudo apt-get install ros-melodic-mavros*
sudo apt-get install xdotool
sudo apt-get install ros-melodic-mavros-msgs

# Build workspace
cd catkin_ws
catkin_make

# Setup PX4
cd ~/VTNMPC-Autonomous-Wind-Turbine-Inspection
./install_dependencies_and_setup_px4_modified.sh
# Ignore errors related to python 2.7

# Add aliases for arming and setting mode
sudo gedit ~/.bashrc
# Add the following lines:
# alias arm='rosrun mavros mavsafety arm'
# alias disarm='rosrun mavros mavsafety disarm'
# alias offboard='rosrun mavros mavsys mode -c OFFBOARD'

# Starting the simulation
cd ~/VTNMPC-Autonomous-Wind-Turbine-Inspection/WTI_px4_modified/shell_scripts/
./run_sitl_gazebo_withWrapper_terminator.sh matrice_100

# Running the Inspection Planner
roslaunch dji_m100_trajectory m100_trajectory_v2_indoor.launch

# Activate traj_on

# In another terminal, arm the drone and set the mode to offboard
arm
offboard

# Launching the VT-NMPC
roslaunch quaternion_point_traj_nmpc quaternion_point_traj_nmpc.launch

# Adding wind to the simulation
roslaunch dji_m100_trajectory windgen_recdata.launch

# Running the whole inspection framework
# Bring the drone to the initial position (-3, 0, 3)
# Run the point and normal generator node
rosrun dji_m100_trajectory GP_statemachine

# Change the mode to GP (Global Planner) and tick the 'point to inspect' checkbox


# Citation
If you use this framework in your work, please cite the following paper:
[Link to paper](https://ieeexplore.ieee.org/abstract/document/10406329?casa_token=E3nhD10-h10AAAAA:aJ4iZipbMURKRQeuiITx9BC0teooc_D5BAb71Vfi4Cw4mCCRAa4WPwd7oUVGSTe0xXLezf0lSw) 
```bash
@inproceedings{amer2023visual,
  title={Visual Tracking Nonlinear Model Predictive Control Method for Autonomous Wind Turbine Inspection},
  author={Amer, Abdelhakim and Mehndiratta, Mohit and le Fevre Sejersen, Jonas and Pham, Huy Xuan and Kayacan, Erdal},
  booktitle={2023 21st International Conference on Advanced Robotics (ICAR)},
  pages={431--438},
  year={2023},
  organization={IEEE}
}
```




