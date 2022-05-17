close all
clear all
clc

set(gcf,'color','w');
%M = dlmread('inspection_1.txt');
M = dlmread('Results_1/test3.txt');
mx = dlmread('MeshX.txt');
my = dlmread('MeshY.txt');
mz = dlmread('MeshZ.txt');
%C = dlmread('coverage.txt');
%C1 = dlmread('hakim.txt');
ts=1550;
grid on
%yawset(gcf,'color','w');
figure
M=M(ts:end,:);
x=M(:,5:7);
p=M(:,8:10);
n=M(:,11:13);
x_ref=M(:,2:4);
a= p-x;
a_ref=p-x_ref;
yaw=M(:,19);
yaw=yaw*pi/180;
for i=1:length(a)
s1(i,1)=(cos(yaw(i))*a(i,1)+sin(yaw(i))*a(i,2))/norm(a(i,1:2));
s2(i,1)=norm(a(i,1:2));
s22(i,1)=norm(a_ref(i,1:2));
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

figure 

set(gcf,'color','w');

plot(t,s1, 'LineWidth', 3.0,'LineSmoothing', 'on');

figure
set(gcf,'color','w');
plot(t,s22, 'k','LineWidth', 3.0,'LineSmoothing', 'on');
hold on
plot(t,s2,'b','LineWidth', 3.0,'LineSmoothing', 'on');
hold on
yline(10,'--','r','LineWidth', 3.0);

legend('Waypoint trajectory','MPC obtained trajectory', 'Safe Distance')
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




