
close all
M_PA = dlmread('/home/airlab/hakim_ws/src/WTI_catkin/Results/Data/perception_test/pampc.txt');    %VTMPC data
M_VT = dlmread('/home/airlab/hakim_ws/src/WTI_catkin/Results/Data/perception_test/vtnmpc.txt');    %VTMPC data

psi=[0 0 0 30 0 60 0 90 0 120 0 150 0 180];
yaw_vt=M_VT(:,19);
yaw_pa=M_PA(:,19);


figure

plot (yaw_vt(2000:end),'b')

hold on
plot (yaw_pa(4000:end))
