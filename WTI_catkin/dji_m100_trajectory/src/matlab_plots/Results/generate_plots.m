close all
clear all
clc

set(gcf,'color','w');
%M = dlmread('inspection_1.txt');
M = dlmread('Results_costs/test1.txt');
mx = dlmread('MeshX.txt');
my = dlmread('MeshY.txt');
mz = dlmread('MeshZ.txt');


nx_160 = dlmread('/home/hakim/catkin_ws/src/WTI_catkin/Results/Data/normals/mp_d1cm_interp_n_x.txt');
ny_160 = dlmread('/home/hakim/catkin_ws/src/WTI_catkin/Results/Data/normals/mp_d1cm_interp_n_y.txt');
nz_160 = dlmread('/home/hakim/catkin_ws/src/WTI_catkin/Results/Data/normals/mp_d1cm_interp_n_z.txt');

px_160 = dlmread('/home/hakim/catkin_ws/src/WTI_catkin/Results/Data/Point_to_View_Trajectory/mp_d1cm_interp_x.txt');
py_160 = dlmread('/home/hakim/catkin_ws/src/WTI_catkin/Results/Data/Point_to_View_Trajectory/mp_d1cm_interp_y.txt');
pz_160 = dlmread('/home/hakim/catkin_ws/src/WTI_catkin/Results/Data/Point_to_View_Trajectory/mp_d1cm_interp_z.txt');


pointx_160 = dlmread('/home/hakim/catkin_ws/src/WTI_catkin/Results/Data/Point_to_View_Trajectory/mp_d1cm_interp_x.txt');
pointy_160 = dlmread('/home/hakim/catkin_ws/src/WTI_catkin/Results/Data/Point_to_View_Trajectory/mp_d1cm_interp_y.txt');
pointz_160 = dlmread('/home/hakim/catkin_ws/src/WTI_catkin/Results/Data/Point_to_View_Trajectory/mp_d1cm_interp_z.txt');


path=dlmread('path_half.txt');

xq=[0:0.032:160]
%px_5000_o = interp1([1:length(px_160)],px_160,xq);
%py_5000_o = interp1([1:length(px_160)],py_160,xq);
%pz_5000_o = interp1([1:length(px_160)],pz_160,xq);


px_5000 = dlmread('px_fit_interp_5000.txt');
py_5000 = dlmread('py_fit_interp_5000.txt');
pz_5000 = dlmread('pz_fit_interp_5000.txt');


px_5000_2 = dlmread('px_fit_interp_2_5000.txt');
py_5000_2 = dlmread('py_fit_interp_2_5000.txt');
pz_5000_2 = dlmread('pz_fit_interp_2_5000.txt');






px = dlmread('px_s.txt');
py = dlmread('py_s.txt');
pz = dlmread('pz_s.txt');

%C = dlmread('coverage.txt');
%C1 = dlmread('hakim.txt');
ts=1;
grid on
%yawset(gcf,'color','w');
figure
M=M(ts:end,:);
x=M(:,5:7);
p=M(:,8:10);
n=M(:,11:13);
kkt=M(:,26);
obj=M(:,27);




velocity=M(:,14:16);

vel_d=[0 0 0; diff(px_160)/0.125,diff(py_160)/0.125,diff(pz_160)/0.125]; 



angles_d=M(:,17:19);
rates_d=M(:,28:30);
x_ref=M(:,2:4);
a= p-x;
a_ref=p-x_ref;
yaw=M(:,19);
yaw=yaw*pi/180;
for i=1:length(a)
s1(i,1)=(cos(yaw(i))*a(i,1)+sin(yaw(i))*a(i,2))/norm(a(i,1:2));
s2(i,1)=(a(i,1)^2+a(i,2)^2)^0.5;
s22(i,1)=norm(a_ref(i,1:2));
s3(i,1)=a(i,1)*n(i,1)+a(i,1)*n(i,2);
s4(i,1)=a(i,1)*n(i,2)-a(i,2)*n(i,1);
end

s3=a(:,1).*n(:,1)+a(:,2).*n(:,2);

ez=abs(x(:,3)-x_ref(:,3));

t=M(:,1)-M(ts,1);
%plot3(M(1321:26954,8),M(1321:26954,9),M(1321:26954,10),'k', 'LineWidth', 3.0,'LineSmoothing', 'on');
set(gcf,'color','w');
grid minor
%xlim([0 510])
plot3(x(:,1),x(:,2),x(:,3),'k', 'LineWidth', 3.0,'LineSmoothing', 'on');
hold on
plot3(x_ref(:,1),x_ref(:,2),x_ref(:,3),'g', 'LineWidth', 3.0,'LineSmoothing', 'on');
hold on
patch(mx'+68,my'-32,mz'-70,'r','EdgeColor','k','FaceAlpha',0.7);
set(gcf,'color','w');

figure 

set(gcf,'color','w');

plot(t,s1, 'b', 'LineWidth', 3.0,'LineSmoothing', 'on');

xlabel('time [s]');
ylabel('s1 objective []');
xlim([0 3000])
%legend('s1 objective')
%% Distance Plot
figure
set(gcf,'color','w');
%plot(t,s22, 'k','LineWidth', 3.0,'LineSmoothing', 'on');
hold on
plot(t,s2,'b','LineWidth', 3.0,'LineSmoothing', 'on');
hold on
yline(10,'--','LineWidth', 3.0,'Color',[0 0 0.1]);

yline(11.5,'--','LineWidth', 3.0);

xlim([50 1100])

text(490,10.08,'safe distance')
text(500,11.43,'maximum distance')


%legend('MPC obtained trajectory', 'Minimum Safe Distance', 'Max distance')
xlabel('time [s]');
ylabel('Distance from turbine [m]');
figure 

plot(t(1:20:end),s1(1:20:end)*100, 'k', 'LineWidth', 3.0,'LineSmoothing', 'on');
hold on
xlabel('time [s]');
ylabel('Coverage %');
count=0;
for i=1:length(p)-1
   
 if ((p(i,1)-p(i+1,1)) ~= 0 ||  (p(i,2)-p(i+1,2)) ~= 0 || (p(i,3)-p(i+1,3)) ~= 0) && count> 100
     event(i)=1;
     count=0;
 else
     event(i)=0;
     count=count+1;
 end
     
 
 
     
end
event(i+1)=0;
figure

set(gcf,'color','w');

plot3(x_ref(:,1),x_ref(:,2),x_ref(:,3),'g', 'LineWidth', 3.0,'LineSmoothing', 'on');
hold on
scatter3(x_ref(:,1),x_ref(:,2),x_ref(:,3),10,s22, 'filled');

figure

set(gcf,'color','w');

plot3(x_ref(:,1)-68,x_ref(:,2)+32,x_ref(:,3)+70,'k', 'LineWidth', 3.0,'LineSmoothing', 'on');
hold on
scatter3(x(:,1)-68,x(:,2)+32,x(:,3)+70,10,s2, 'filled');
hold on
patch(mx',my',mz','w','EdgeColor','k','FaceAlpha',0.7);
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
legend('Waypoint trajectory','MPC obtained trajectory')
cb1 = colorbar(); 
ylabel(cb1, 'distance from turbine [m]')


%scatter(t(1:50:end), s1(1:50:end))
figure 

plot(t(1:20:end),-s3(1:20:end), 'r', 'LineWidth', 3.0,'LineSmoothing', 'on');
hold on
plot(t(1:20:end),s2(1:20:end), 'b', 'LineWidth', 3.0,'LineSmoothing', 'on');

xlabel('time [s]');
ylabel('s3');

figure
set(gcf,'color','w');

%xlim([0 510])

plot3(p(:,1),p(:,2),p(:,3),'b', 'LineWidth', 3.0,'LineSmoothing', 'on');
hold on
%plot3(x_ref(:,1),x_ref(:,2),x_ref(:,3),'w', 'LineWidth', 3.0,'LineSmoothing', 'on');
hold on
patch(mx'+68,my'-32,mz'-70,'k','EdgeColor','w','FaceAlpha',0.7);
pbaspect([1 12 12])
axis off
set(gcf,'color','w');
figure
plot (t,p(:,1),'r')
hold on
plot (t,p(:,2),'b')
hold on	
plot (t,p(:,3),'k')
hold on

for i =2:length(p)
vx(i)=(p(i,1)-p(i-1,1))/0.01;
vy(i)=(p(i,2)-p(i-1,2))/0.01;
vz(i)=(p(i,3)-p(i-1,3))/0.01;

end

figure
plot (t,vx,'r')
hold on
plot (t,vy,'b')
hold on	
plot (t,vz,'k')
hold on

figure
plot (t,x(:,1),'k','LineWidth', 2.0)
hold on
plot (t,x(:,2),'b','LineWidth', 2.0)
%hold on	
%plot (t,x(:,3),'k')

hold on
plot (t,p(:,1),'r')
hold on
plot (t,p(:,2),'g')

figure 
plot (t, s2)
xlim([30 1200])

%% Plotting the 2-D Trajectory 
figure 
set(gcf,'color','w');
xlim([50 1100])
hold on
plot (t,p(:,1),'--','Color',[0 0 0],'LineWidth', 2.0)
hold on

plot (t,x(:,1),'-','Color',[0 0 0],'LineWidth', 2.0)

xlabel('time [s]') 
ylabel('distance [m]') 

hold on
xlim([50 1100])
legend('p_x','x')

figure
set(gcf,'color','w');

plot (t,p(:,2),'--','Color',[1 0 0],'LineWidth', 2.0)
hold on

plot (t,x(:,2),'-','Color',[1 0 0],'LineWidth', 2.0)
hold on
xlim([50 1100])
legend('p_y','y')

xlabel('time [s]') 
ylabel('distance [m]') 
figure
set(gcf,'color','w');
xlim([50 1100])
plot (t,p(:,3),'--','Color',[0 0 1],'LineWidth', 2.0)
hold on
xlim([50 1100])
plot (t,x(:,3),'-','Color',[0 0 1],'LineWidth', 2.0)



xlabel('time [s]') 
ylabel('distance [m]') 


legend('p_z','z')



%% Plotting costs 

figure
set(gcf,'color','w');

xlim([50 1100])
C_h= 80*(s1-1).^2;
C_d= 200*(s2-10).^2;
C_r=  60*(s3+12.5).^2;



plot (t,C_h,'-','Color',[0 0 1],'LineWidth', 2.0)
hold on
plot (t,C_d,'-','Color',[0 1 0],'LineWidth', 2.0)
hold on
plot (t,C_r,'-','Color',[1 0 0],'LineWidth', 2.0)
xlim([50 1100])
xlabel('time [s]') 
ylabel('Cost') 
legend('C_h','C_d', 'C_r')


figure 
set(gcf,'color','w');
plot (t,s4,'-','Color',[0 0 1],'LineWidth', 2.0)
xlim([50 1100])
xlabel('time [s]') 
ylabel('s4') 




figure 
set(gcf,'color','w');
plot (t,kkt,'-','Color',[0 0 1],'LineWidth', 2.0)
xlim([50 1100])
xlabel('time [s]') 
ylabel('KKT') 

figure
set(gcf,'color','w');
plot (t,obj,'-','Color',[0 0 1],'LineWidth', 2.0)
xlim([50 1100])
xlabel('time [s]') 
ylabel('Objective') 


 figure
set(gcf,'color','w');
plot (1:length(vel_d),vel_d(:,1),'-','Color',[0 0 1],'LineWidth', 2.0)

hold on
plot (1:length(vel_d),vel_d(:,2),'-','Color',[1 0 0],'LineWidth', 2.0)

hold on
plot (1:length(vel_d),vel_d(:,3),'-','Color',[0 1 0],'LineWidth', 2.0)



vel_abs=(vel_d(:,1).^2+vel_d(:,2).^2+vel_d(:,3).^2).^0.5;


velocity_abs=(velocity(:,1).^2+velocity(:,2).^2+velocity(:,3).^2).^0.5;


xlim([50 1100])
xlabel('time [s]') 
ylabel('velocity') 

legend('vx','vy', 'vz')

figure 
%% Plot Velocities

figure 
hold on
plot (1:length(vel_abs),vel_abs,'-','Color',[0 0 1],'LineWidth', 2.0)
xlabel('time [s]')
ylabel('velocity [m/s]')

xlim ([20 1050])
%% Plot point trajectory
figure
plot3(px,py,pz,'b', 'LineWidth', 3.0,'LineSmoothing', 'on');
xlabel('x')
ylabel('y')
zlabel('z')


%% Plot interpolatted
figure
plot3(px_5000,py_5000,pz_5000,'k', 'LineWidth', 3.0);
hold on 
plot3(px_160,py_160,pz_160,'b', 'LineWidth', 3.0);
pbaspect([1 10 10])

xlabel('x')
ylabel('y')
zlabel('z')

%% plot 2-D interpolated

figure 
hold on
plot (linspace(0,1,length(px_5000)),px_5000,'-','Color',[0 0 1],'LineWidth', 2.0)
xlabel('time [s]')
ylabel('x [m]')

hold on 


%plot ([1:length(px)],px,'--','Color',[0 0 0],'LineWidth', 2.0)



hold on 


plot (linspace(0,1,length(px_160)),px_160,'--','Color',[1 0 0],'LineWidth', 2.0)


hold on

plot (linspace(0,1,length(px_5000_2)),px_5000_2,'-','Color',[0 0.5 0.5],'LineWidth', 2.0)
xlabel('time [s]')
ylabel('x [m]')








figure 
hold on
plot ([1:length(py_5000)],py_5000,'-','Color',[0 0 1],'LineWidth', 2.0)

hold on 


plot ([1:length(py)],py,'--','Color',[0 0 0],'LineWidth', 2.0)



hold on 


plot ([1:length(py_160)],py_160,'--','Color',[1 0 0],'LineWidth', 2.0)


hold on

plot ([1:length(py_5000_2)],py_5000_2,'-','Color',[0 0.5 0.5],'LineWidth', 2.0)
xlabel('time [s]')
ylabel('y [m]')


figure 
hold on
plot ([1:length(pz_5000)],pz_5000,'-','Color',[1 0 0],'LineWidth', 2.0)
xlabel('time [s]')
ylabel('z [m]')


close all
%px_inter = interp1(1:length(px_160), px_160, linspace(1, length(px_160), 10000), 'cubic');
%py_inter = interp1(1:length(py_160), py_160, linspace(1, length(py_160), 10000), 'cubic');
%pz_inter = interp1(1:length(pz_160), pz_160, linspace(1, length(pz_160), 10000), 'cubic');

px_inter=px_160;
py_inter=py_160;
pz_inter=pz_160;




px_inter_s = interp1(1:length(px_160), px_160, linspace(1, length(px_160), 10000), 'spline');
py_inter_s = interp1(1:length(py_160), py_160, linspace(1, length(py_160), 10000), 'spline');
pz_inter_s = interp1(1:length(pz_160), pz_160, linspace(1, length(pz_160), 10000), 'spline');

nx_inter = interp1(1:length(nx_160), nx_160, linspace(1, length(nx_160), length(px_inter)));
ny_inter = interp1(1:length(ny_160), ny_160, linspace(1, length(ny_160), length(px_inter)));
nz_inter = interp1(1:length(nz_160), nz_160, linspace(1, length(nz_160), length(px_inter)));


for i=1:length(path)/2
path([i],:) = [];
end


wpx_inter = interp1(1:length(path(:,1)), path(:,1), linspace(1, length(path(:,1)),  length(px_inter)) );
wpy_inter = interp1(1:length(path(:,2)), path(:,2), linspace(1, length(path(:,2)),  length(px_inter)) );
wpz_inter = interp1(1:length(path(:,3)), path(:,3), linspace(1, length(path(:,3)),  length(px_inter)));

wp_inter(:,1)=wpx_inter;
wp_inter(:,2)=wpy_inter;
wp_inter(:,3)=wpz_inter;


wp_inter(:,1)=smooth(wp_inter(:,1),100);
wp_inter(:,2)=smooth(wp_inter(:,2),100);
wp_inter(:,3)=smooth(wp_inter(:,3),100);



nx_inter=smooth(nx_inter,300);
ny_inter=smooth(ny_inter,300);
nz_inter=smooth(nz_inter,300);



for i=1:length(nx_inter)
    mag(i)=(nx_inter(i)^2+ny_inter(i)^2)^0.5;
end


for i=1:length(nx_inter)
   nx_inter(i)=nx_inter(i)/mag(i);
   ny_inter(i)=ny_inter(i)/mag(i);
end

%nx_inter=nx_inter./mag;
%ny_inter=ny_inter./mag;

for i=1:length(nx_inter)
    mag(i)=(nx_inter(i)^2+ny_inter(i)^2)^0.5;
end


%nx_inter=smooth(nx_inter,40);
%ny_inter=smooth(ny_inter,40);
%nz_inter=smooth(nz_inter,40);


px_inter_s=smooth(px_inter_s,100);
py_inter_s=smooth(py_inter_s,100);
pz_inter_s=smooth(pz_inter_s,100);


theta_d=atan(ny_inter(:)./nx_inter(:));
theta_d=theta_d*180/pi;


wp_nmpc(:,1)=px_inter+nx_inter*7;
wp_nmpc(:,2)=py_inter+ny_inter*7;
wp_nmpc(:,3)=pz_inter;
%wp_nmpc(:,4)=theta_d;






for i=2:length(px_inter)
    
   % vx_inter(i)=(wp_nmpc(i,1)-wp_nmpc(i-1,1))*80;
   % vy_inter(i)=(wp_nmpc(i,2)-wp_nmpc(i-1,2))*80;
   % vz_inter(i)=(wp_nmpc(i,3)-wp_nmpc(i-1,3))*80;
   vx_inter(i)=(px_inter(i)-px_inter(i-1))*80;
   vy_inter(i)=(py_inter(i)-py_inter(i-1))*80;
   vz_inter(i)=(pz_inter(i)-pz_inter(i-1))*80;

    wp(i,1:3)=0;

end

%for i=1:5
%vx_inter=smooth(vx_inter,100);
%vy_inter=smooth(vy_inter,100);
%vz_inter=smooth(vz_inter,100);
%end


%vx_inter=smooth(vx_inter,100);
%vy_inter=smooth(vy_inter,100);
%vz_inter=smooth(vz_inter,100);
%vx_inter=smooth(vx_inter,100);
%vy_inter=smooth(vy_inter,100);
%vz_inter=smooth(vz_inter,100);


plot ([1:length(wpx_inter)],wp_inter(:,1),'-','Color',[1 0 0],'LineWidth', 2.0)
hold on
plot ([1:length(wpy_inter)],wp_inter(:,2),'-','Color',[0 0 1],'LineWidth', 2.0)
hold on
plot ([1:length(wpz_inter)],wp_inter(:,3),'-','Color',[0 1 0],'LineWidth', 2.0)







%% Remove outliers

a=300;

vx_old=vx_inter;
vy_old=vy_inter;
for i=a+1:length(vy_inter)-a+1
   
    if abs(vy_inter(i))>0.2
        if abs(vy_inter(i)-vy_inter(i-a))>0.2 && abs(vy_inter(i)-vy_inter(i+a))>0.2
         vy_inter(i)=0;
        end 
    end
    
    
end


for i=a+1:length(vx_inter)-a+1
   
    if abs(vx_inter(i))>0.2
        if abs(vx_inter(i)-vx_inter(i-a))>0.2 && abs(vx_inter(i)-vx_inter(i+a))>0.2
         vx_inter(i)=0;
        end 
    end
    
    
end



%% Smoothing
px_inter=smooth(px_inter,50);
py_inter=smooth(py_inter,50);
pz_inter=smooth(pz_inter,50);




%% yaw angle
for i=1:length(nx_inter)
   if (0.5 <= nx_inter(i))   &&  (nx_inter(i) <=1)
       yaw_a(i)=180;
       
   end
   
   if (-0.5 >= nx_inter(i))   &&  (nx_inter(i) >=-1)
       yaw_a(i)=360;
       
   end
   
    if (0.5 <= ny_inter(i))   &&  (ny_inter(i) <=1)
       yaw_a(i)=270;
       
    end
    
    if (-0.5 >= ny_inter(i))   &&  (ny_inter(i) >=-1)
       yaw_a(i)=90+360;
       
    end
    
    if i>=36624
        yaw_a(i)= 360+180;
    end
end
yaw_a=smooth(yaw_a,300);
wp_nmpc(:,4)=yaw_a;



writematrix(px_inter, "px_inter.txt");

writematrix(py_inter, "py_inter.txt");

writematrix(pz_inter, "pz_inter.txt");

writematrix(nx_inter, "nx_inter.txt");

writematrix(ny_inter, "ny_inter.txt");

writematrix(nz_inter, "nz_inter.txt");

writematrix(vx_inter', "vx_inter.txt");

writematrix(vy_inter', "vy_inter.txt");

writematrix(vz_inter', "vz_inter.txt");


writematrix(wp, "wp.txt");
writematrix(wp_nmpc, "wp_inter.txt");



vel_abs=(vx_inter.^2+vy_inter.^2+vz_inter.^2).^0.5;

plot ([1:length(wpx_inter)],wpx_inter,'-','Color',[0 1 0],'LineWidth', 2.0)


close all

figure
b=1;
a=162;

plot3(pointx_160(b:a) ,pointy_160(b:a) ,pointz_160(b:a) )
hold on
patch(mx',my',mz','r','EdgeColor','k','FaceAlpha',0.7);
figure 
plot ([1:length(wpz_inter)],py_inter,'-','Color',[0 1 0],'LineWidth', 2.0)
hold on
plot ([1:length(vy_inter)],py_inter,'-','Color',[0 0 1],'LineWidth', 2.0)
close all

plot ([1:length(vy_old)],vy_old)
hold on
plot ([1:length(vy_inter)],vy_inter)
close all
plot ([1:length(vx_old)],vx_old)
hold on
plot ([1:length(vx_inter)],vx_inter)


plot3(px_inter ,py_inter ,pz_inter )























