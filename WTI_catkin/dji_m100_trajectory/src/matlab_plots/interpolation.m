
close all
px = dlmread('/home/hakim/catkin_ws/src/WTI_catkin/dji_m100_trajectory/src/matlab_plots/Generating_normals/px.txt');
py= dlmread('/home/hakim/catkin_ws/src/WTI_catkin/dji_m100_trajectory/src/matlab_plots/Generating_normals/py.txt');
pz = dlmread('/home/hakim/catkin_ws/src/WTI_catkin/dji_m100_trajectory/src/matlab_plots/Generating_normals/pz.txt');


nx = dlmread('/home/hakim/catkin_ws/src/WTI_catkin/dji_m100_trajectory/src/matlab_plots/Generating_normals/nx.txt');
ny= dlmread('/home/hakim/catkin_ws/src/WTI_catkin/dji_m100_trajectory/src/matlab_plots/Generating_normals/ny.txt');
nz = dlmread('/home/hakim/catkin_ws/src/WTI_catkin/dji_m100_trajectory/src/matlab_plots/Generating_normals/nz.txt');




%px_1000 = dlmread('/home/hakim/catkin_ws/src/WTI_catkin/dji_m100_trajectory/src/matlab_plots/Generating_normals/px.txt');
%py_1000= dlmread('/home/hakim/catkin_ws/src/WTI_catkin/dji_m100_trajectory/src/matlab_plots/Generating_normals/py.txt');
%pz_1000 = dlmread('/home/hakim/catkin_ws/src/WTI_catkin/dji_m100_trajectory/src/matlab_plots/Generating_normals/pz.txt');




%[xq,yq] = meshgrid(-70:-.02:-83, -50:.02:50);
%z4 = griddata(px,py,pz,xq,yq,'cubic');


%figure
%plot3(px,py,pz,'mo')
%hold on
%mesh(xq,yq,z4)
%title('Cubic')
%legend('Sample Points','Interpolated Surface','Location','NorthWest')



px_s = smooth(px,120);
py_s = smooth(py,120);
pz_s = smooth(pz,120);


nx_s = smooth(nx,120);
ny_s = smooth(ny,120);
nz_s = smooth(nz,120);



figure 
plot ([1:length(px_s)],px_s,'r')

hold on

plot ([1:length(px_s)],px,'b')

figure 

plot ([1:length(px_s)],py_s,'r')

hold on

plot ([1:length(px_s)],py,'b')
figure 

plot ([1:length(px_s)],pz_s,'r')

hold on

plot ([1:length(px_s)],pz,'b')






figure 
plot ([1:length(nx_s)],nx_s,'r')

hold on

plot ([1:length(nx_s)],nx,'b')

figure 

plot ([1:length(nx_s)],ny_s,'r')

hold on

plot ([1:length(nx_s)],ny,'b')
figure 

plot ([1:length(nx_s)],nz_s,'r')

hold on

plot ([1:length(nx_s)],nz,'b')



for i =2:length(px_s)


    
vx(i)=(px_s(i)-px_s(i-1))/1;
vy(i)=(py_s(i)-py_s(i-1))/1;
vz(i)=(pz_s(i)-pz_s(i-1))/1;


end
vx_s = smooth(vx,120);
vy_s = smooth(vy,120);
vz_s = smooth(vz,120);

figure

plot ([1:length(vx)],vx,'r')

figure

plot ([1:length(vy)],vy,'r')

figure

plot ([1:length(vz)],vz,'r')

vel_abs=(vx.^2+vy.^2+vz.^2).^0.5;
vel_abs_s=(vx_s.^2+vy_s.^2+vz_s.^2).^0.5;
figure

plot ([1:length(vz)],vel_abs,'r')

hold on
plot ([1:length(vz)],vel_abs_s,'b')

writematrix(px_s, "px_s.txt");

writematrix(py_s, "py_s.txt");

writematrix(pz_s, "pz_s.txt");

writematrix(nx_s, "nx_s.txt");

writematrix(ny_s, "ny_s.txt");

writematrix(nz_s, "nz_s.txt");

writematrix(vx, "vx_s.txt");

writematrix(vy, "vy_s.txt");

writematrix(vz, "vz_s.txt");
