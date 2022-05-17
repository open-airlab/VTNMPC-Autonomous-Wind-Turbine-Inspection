clear all


red_vt = dlmread('/home/hakim/catkin_ws/src/WTI_catkin/Trajectory_generation_coverage/python_scripts-master/data/in/vtmpc_redundancy.txt');    %VTMPC data
red_nmpc = dlmread('/home/hakim/catkin_ws/src/WTI_catkin/Trajectory_generation_coverage/python_scripts-master/data/in/nmpc_red.txt');    %VTMPC data


red_sum= sum(red_vt(:,2));
red_sum=red_sum/1002;


red_sum2= sum(red_nmpc(:,2));
red_sum2=red_sum2/1002;