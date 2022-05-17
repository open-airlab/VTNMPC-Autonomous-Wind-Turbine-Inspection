close all
clc
clear all

%input data
%M = dlmread('/home/hakim/catkin_ws/src/WTI_catkin/dji_m100_trajectory/src/matlab_plots/Results/Coverage_criteria/Results_2/test2.txt');
%M = dlmread('/home/hakim/catkin_ws/src/WTI_catkin/dji_m100_trajectory/src/matlab_plots/Results/Results_vel_1/test1.txt');
M = dlmread('/home/hakim/catkin_ws/src/WTI_catkin/dji_m100_trajectory/src/matlab_plots/Results/Results_PAMPC_new/PAMPC_data.txt');
M = dlmread('/home/hakim/catkin_ws/src/WTI_catkin/dji_m100_trajectory/src/matlab_plots/Results/Results_VTMPC_new/VTMPC_results.txt');

%mx = dlmread('triang_x.txt');
%my = dlmread('triang_y.txt');
%mz= dlmread('triang_z.txt');
%mx = dlmread('MeshX.txt');
%my = dlmread('MeshY.txt');
%mz = dlmread('MeshZ.txt');

Mx = dlmread('MeshX.txt');
My = dlmread('MeshY.txt');
Mz = dlmread('MeshZ.txt');

mx = dlmread('mesh_x.txt');
my = dlmread('mesh_y.txt');
mz = dlmread('mesh_z.txt');



px =dlmread('px_160.txt');
py =dlmread('py_160.txt');
pz =dlmread('pz_160.txt');
nx =dlmread('nx_160.txt');
ny =dlmread('ny_160.txt');
nz =dlmread('nz_160.txt');




mx=mx';
my=my';
mz=mz';

d_min=10000;

for j=1:160
for i=1:length(mx) 
 dx=(px(j)-mx(i,1))^2 +(px(j)-mx(i,2))^2  +(px(j)-mx(i,3))^2;
dy=(py(j)-my(i,1))^2 +(py(j)-my(i,2))^2  +(py(j)-my(i,3))^2;
dz=(pz(j)-mz(i,1))^2 +(pz(j)-mz(i,2))^2  +(pz(j)-mz(i,3))^2;

dt=(dx+dy+dy)^0.5;
if dt<d_min
   
mx_o(j,:)= mx(i,:);
my_o(j,:)= my(i,:);
mz_o(j,:)= mz(i,:);
d_min=dt;
n_m(j)=i;
end


   
end 
d_min=10000;
end
mx=mx';
my=my';
mz=mz';
mx_o=mx_o';
my_o=my_o';
mz_o=mz_o';

%plot3(px(500:end),py(500:end),pz(500:end),'k', 'LineWidth', 2.0,'LineSmoothing', 'on');

patch(mx,my,mz, 'red','EdgeColor','k','FaceAlpha',0.7);

%patch(mx_o,my_o,mz_o, 'red','EdgeColor','k','FaceAlpha',0.7);


%patch(mx_o,my_o,mz_o, 'red','EdgeColor','k','FaceAlpha',0.7);







mx=mx';
my=my';
mz=mz';
psi=M(:,19);
x=M(:,5:7);
x_ref=M(:,2:4);

%px=mx(:,1)+mx(:,1)+mx(:,1);

px=px;
py=py;
pz=pz;




%s=a(:,1).*n(:,1)+a(:,2).*n(:,2);





x(:,1)=x(:,1)-68; x(:,2)=x(:,2)+32;
x(:,3)=x(:,3)+70;
%camera

d=10000;


%ax=px-nx;
%ay=py-ny;
hh=0;%
out=zeros(162);
for j=1:length(px)
for i=1:length(x)

    
    %d1= ((x(i,1)-px(j))^2+(x(i,2)-py(j))^2+(x(i,3)-pz(j))^2)^0.5;
    d1= ((x(i,1)-px(j))^2+(x(i,2)-py(j))^2)^0.5;
    
    
    %d1= ((x(i,1)-px(j))^2+(x(i,2)-py(j))^2)^0.5;
    s3i=((px(j)-x(i,1))*nx(j)+(py(j)-x(i,2))*ny(j));
    s4=((px(j)-x(i,1))*ny(j)-(py(j)-x(i,2))*nx(j));

    if ( ((d1-10.5)^2<(d-10.5)^2) && ((px(j)-x(i,1))*nx(j)+(py(j)-x(i,2))*ny(j))<-2 && d1>2  && (x(i,3)-pz(j))^2<0.5)   
        %&& ((x(i,3)-pz(j))^2)^0.5<1
        d=d1;
        n=i;  
        hh=n;
        s3(j)=(px(j)-x(i,1))*nx(j)+(py(j)-x(i,2))*ny(j);
        s_p(j)=((px(j)-x(i,1))*ny(j)+(py(j)-x(i,2))*nx(j))/d;
        dist(j)=d;
        
        
    end  

end 

if n==0
   n=hh;
end





%ax(j)=px(j)-x(n,1);
%ay(j)=py(j)-x(n,2);
%az(j)=0;
nn(j)=n;

dd(j)=d;
x1(j,:)=x(n,:);
psi1(j,:)=psi(n,:);
x_ref1(j,:)=x_ref(n,:);
%s_p(j)=((px(j)-x(n,1))*ny(j)-(py(j)-x(n,2))*nx(j));
d=1000000;
n=0;
end





%plot3(x1(:,1),x1(:,2),x1(:,3),'k', 'LineWidth', 3.0,'LineSmoothing', 'on');





%sensor_s=8.5*0.0001*2;    %sensor size

%pixel_s=0.011;
pixel_s=0.002;
%f=2000*pixel_s;

f=35;

%f=20;
%focal length
res=1000;
%sensor_s=0.002*res*2;
%sensor_s=0.011;
sensor_s=8.4/2;
pitch= 0*pi/180;
%yaw=0*pi/180;
roll=0*pi/180;

%for i=1:length(px)
count=0;
%for i=1:length(px)
%for j=1:length(x1)
%for j=1:162

%% Camera parameters
f=16; %15-50 mm
sensor_x= 23.5;  %mm
sensor_y= 15.7; %mm
count1=1;
for i=97:97

 p_c= [px(i);py(i);pz(i);1];
 p1=[mx(i,1);my(i,1);mz(i,1);1];
 p2=[mx(i,2);my(i,2);mz(i,2);1];
 p3=[mx(i,3);my(i,3);mz(i,3);1];
 yaw=psi1(i)*pi/180;
 
 %yaw=1;
 %roll=yaw;
 %yaw=0;
 %yaw=pi;
 drone_x=x1(i,:)';
% drone_x=x_ref1(i,:)';
 %yaw=pi;
 %yaw=0;
 
 
 
 
 %%trial
 
 %p_c=[10;0;0;1];
 %drone_x=[3;0;0];
 %yaw=30*pi/180;
 %pitch=-90*pi/180;
 
 %drones position

%t=[0;0;0;0];

rM=[1    0           0;
    0    cos(roll)   sin(roll);
    0    -sin(roll)  cos(roll)];
pM=[cos(pitch)      0       sin(pitch);
    0               1       0;
   -sin(pitch)      0       cos(pitch)];
yM=[cos(yaw)        -sin(yaw)        0;
    sin(yaw)       cos(yaw)        0;
    0               0               1];



%DCM=(rM*pM)*yM;
%DCM=(rM*yM)*pM;

DCM=yM*pM*rM;   %XYZ
%DCM=rM*pM*yM;
%DCM=rM*(pM)*yM;
%DCM=(yM)*pM;
%DCM=(pM*1)*rM;

R=DCM;

t= drone_x;
%t=[0 ;0; 0];
t=[t;1];


RT(1:3,1:3)=R;


RT(1:4,4)=t;
%RT(1:4,4)=t;
RT(4,1:4)=[0 0 0 1];

%RT=inv(RT);
%RT(4,1:4)=[0 0 0 1];
%RT=inv(RT);

xx=R*drone_x;
xx=[xx;0];

%pt_c=RT*(p_c-t)-t;  %point in local coordinates
xx=[drone_x;0];

RT=inv(RT);
pt_c=RT*(p_c);

pt_1=RT*(p1);
pt_2=RT*(p2);
pt_3=RT*(p3);

RT_inv=inv(RT);
inv_v=RT_inv*[14;0;0;1];

pt_vx(i)=pt_c(1);

pt_vy(i)=pt_c(2);

pt_vz(i)=pt_c(3);
%pt_c
z=pt_c(1);
%pt_c(3)=pt_c(1);
%pt_c(1)=pt_cz;
u_test=f*(pt_c(2)/z);
v_test=f*(pt_c(3)/z);


u_c=f*(pt_c(2)/pt_c(1));
v_c=f*(pt_c(3)/pt_c(1));

u_1=f*(pt_1(2)/pt_1(1));
v_1=f*(pt_1(3)/pt_1(1));

u_2=f*(pt_2(2)/pt_2(1));
v_2=f*(pt_2(3)/pt_2(1));


u_3=f*(pt_3(2)/pt_3(1));
v_3=f*(pt_3(3)/pt_3(1));




 % Check if point is behind the drone
  ax= drone_x(1)-px(i);
  ay= drone_x(2)-py(i);
  s_ri=ax*nx(i)+ay*ny(i);

  
  


if  (abs(u_c)>(sensor_x/2) || abs(v_c)>(sensor_y/2) ||   abs(u_1)>(sensor_x/2) || abs(v_1)>(sensor_y/2)) ||...   
    (abs(u_2)>(sensor_x/2) || abs(v_2)>(sensor_y/2) ||abs(u_3)>(sensor_x/2) || abs(v_3)>(sensor_y/2)) 

    out(i)=0;
    count=count+1;
else
    out(i)=1;
end
  if s_ri<0 || pt_c(1)<0
      out(i)=0;
  end

  
if out(i)==0
    kk(count1)=i;
    
    count1=count1+1;
    count1
end
  
end









plot3(x(1200:end,1),x(1200:end,2),x(1200:end,3),'k', 'LineWidth', 2.0,'LineSmoothing', 'on');

hold on

%scatter3(px(10:10),py(10:10),pz(10:10),10,pz(10:10), 'filled');

hold on
patch(mx',my',mz','w','EdgeColor','k','FaceAlpha',0.7);


hold on


%plot3(x1(1:40,1),x1(1:40,2),x1(1:40,3),'r', 'LineWidth', 3.0,'LineSmoothing', 'on');
set(gcf,'color','w');

for i=97:97
    if out(i)==1 
        
        %scatter3(x1(i,1),x1(i,2),x1(i,3),'b','filled', 'LineWidth', 2.0,'DisplayName','hakim');
    
        hold on
        %scatter3(px(i),py(i),pz(i),10, 'k', 'filled');
        hold on
        %quiver3(x1(i,1),x1(i,2),x1(i,3),ax(i)/2,ay(i)/2,az(i),'g')
        %quiver3(nx(i,1),x1(i,2),ny(i,3),10*cos(psi1(i)*pi/180),10*sin(psi1(i)*pi/180),0,'g')
       % quiver3(x1(i,1),x1(i,2),x1(i,3),4*cos(psi1(i)*pi/180),4*sin(psi1(i)*pi/180),0,'b','MaxHeadSize',1000,'LineWidth', 1.5);
        

        patch(mx(i,:),my(i,:),mz(i,:),'b');
        hold on
        if i==j
            patch(mx(i,:),my(i,:),mz(i,:),'g');
        end        
        
        hold on
      % patch(mx(i),my(i),mz(i),'FaceColor','r');
       % legend('path', 'turbine','hakim','aschasc')
        hold on
    else
        
         % scatter3(x1(i,1),x1(i,2),x1(i,3),'k','filled', 'LineWidth',  3.0);
         % quiver3(x1(i,1),x1(i,2),x1(i,3),4*cos(psi1(i)*pi/180),4*sin(psi1(i)*pi/180),0,'k','LineWidth', 1.0)
         scatter3(x1(j,1),x1(j,2),x1(j,3),30, 'red', 'filled');
         patch(mx(i,:),my(i,:),mz(i,:),'red');
       
         quiver3(drone_x(1),drone_x(2),drone_x(3),inv_v(1)-drone_x(1),inv_v(2)-drone_x(2),inv_v(3)-drone_x(3),'g','LineWidth', 2.0)
         if i==j
            patch(mx(i,:),my(i,:),mz(i,:),'r');
        end    
    end
       
end
%legend('hakim')
%legend('path', 'turbine')

%scatter3(x1(:,1),x1(:,2),x1(:,4),'r', 'LineWidth', 3.0);
hold on

%quiver3(px',py',pz',nx',ny',nz','g')


%%%%%%%%%%%%%

for i=1:162
    if -s3(i)<2
     s3(i)=s3(i-1);
     i;
     s3(i);
     dist(i)=dist(i-1);
    end
end
    
figure
tt=1:162;
plot(tt,-s3, 'k', 'LineWidth', 3.0,'LineSmoothing', 'on');
hold on
plot(tt,dist, 'b', 'LineWidth', 3.0,'LineSmoothing', 'on');
hold on
xlabel('time [s]');
ylabel('s3');


%a1=x1(:,1)-p1;
%a2=x2(:,1)-p2;
%a3=x3(:,1)-p3;

%s4=ny*az
sin_sp=asin(s_p)*(180/pi);
figure 
plot(tt,sin_sp, 'k', 'LineWidth', 2.0,'LineSmoothing', 'on');
xlabel('time [s]');
ylabel('Incidence Angle [deg]');



figure
for i=1:length(pt_vx)
scatter3(pt_vx(i),pt_vy(i),pt_vz(i),10,'r', 'filled');
hold on



end
hold on
scatter3(0,0,0,10,'k', 'filled');
hold on
quiver3(0,0,0,10,0,0,'b')
xlabel('x');
ylabel('y');
zlabel('z');
figure 


set(gcf,'color','w');
plot3(px',py',pz','r', 'LineWidth', 2.0,'LineSmoothing', 'on');
hold on
scatter3(px(1),py(1),pz(1),1, 'white', 'filled');
hold on
patch(Mx',My',Mz','w','FaceAlpha',.5,'LineWidth',1,'FaceAlpha',.01,'EdgeColor','k' );

pbaspect([1 10 10])




grid off 
axis off