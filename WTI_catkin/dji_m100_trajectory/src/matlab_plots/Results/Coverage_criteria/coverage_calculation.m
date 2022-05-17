close all
clc
clear all

%input data

M = dlmread('Results_1/test3.txt');
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
psi=M(:,19);
x=M(:,5:7);
%px=mx(:,1)+mx(:,1)+mx(:,1);

px=px;
py=py;
pz=pz;




%s=a(:,1).*n(:,1)+a(:,2).*n(:,2);





x(:,1)=x(:,1)-68;
x(:,2)=x(:,2)+32;
x(:,3)=x(:,3)+70;
%camera

d=10000;


%ax=px-nx;
%ay=py-ny;

for j=1:length(px)
for i=1:length(x)

    
    d1= ((x(i,1)-px(j))^2+(x(i,2)-py(j))^2+(x(i,3)-pz(j))^2)^0.5;
    %d1= ((x(i,1)-px(j))^2+(x(i,2)-py(j))^2)^0.5;
   
    if ((d1-0)^2<(d-0)^2 &&  ((px(j)-x(i,1))*nx(j)+(py(j)-x(i,2))*ny(j))<-6) 
        %&& ((x(i,3)-pz(j))^2)^0.5<1
        d=d1;
        n=i;  
    end  
end 

ax(j)=px(j)-x(n,1);
ay(j)=py(j)-x(n,2);
az(j)=0;
nn(j)=n;

dd(j)=d;
x1(j,:)=x(n,:);
psi1(j,:)=psi(n,:);


d=100000;
n=0;
end





%plot3(x1(:,1),x1(:,2),x1(:,3),'k', 'LineWidth', 3.0,'LineSmoothing', 'on');





%sensor_s=8.5*0.0001*2;    %sensor size
pixel_s=0.002;
f=1000*pixel_s;    %focal length
res=1000;
sensor_s=0.002*res;

pitch=-90*pi/180;
%yaw=0*pi/180;
roll=0*pi/180;

%for i=1:length(px)
count=0;
for i=1:10

 p_c= [px(i);py(i);pz(i);1];
 p1=[mx(i,1);my(i,1);mz(i,1);1];
 p2=[mx(i,2);my(i,2);mz(i,2);1];
 p3=[mx(i,3);my(i,3);mz(i,3);1];
 yaw=-psi1(i)*pi/180;
 
 
 %roll=yaw;
 %yaw=0;
 %yaw=pi;
 drone_x=x1(i,:)';
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
pM=[cos(pitch)      0       -sin(pitch);
    0               1       0;
    sin(pitch)      0       cos(pitch)];
yM=[cos(yaw)        sin(yaw)        0;
    -sin(yaw)       cos(yaw)        0;
    0               0               1];



%DCM=(rM*pM)*yM;
DCM=(rM*yM)*pM;
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


u_c=f*(pt_c(2)/pt_c(3));
v_c=f*(pt_c(1)/pt_c(3));

u_1=f*(pt_1(2)/pt_1(3));
v_1=f*(pt_1(1)/pt_1(3));

u_2=f*(pt_2(2)/pt_2(3));
v_2=f*(pt_2(1)/pt_2(3));


u_3=f*(pt_3(2)/pt_3(3));
v_3=f*(pt_3(1)/pt_3(3));




if  (abs(u_c)>(sensor_s/2) || abs(v_c)>(sensor_s/2) ||   abs(u_1)>(sensor_s/2) || abs(v_1)>(sensor_s/2)) ||...   
    (abs(u_2)>(sensor_s/2) || abs(v_2)>(sensor_s/2) ||abs(u_3)>(sensor_s/2) || abs(v_3)>(sensor_s/2)) 

    out(i)=1;
    count=count+1;
else
    out(i)=0;
end


end








plot3(x(:,1),x(:,2),x(:,3),'k', 'LineWidth', 3.0,'LineSmoothing', 'on');

hold on

scatter3(px(10:10),py(10:10),pz(10:10),10,pz(10:10), 'filled');

hold on
patch(mx',my',mz','w','EdgeColor','k','FaceAlpha',0.7);


hold on


%plot3(x1(1:40,1),x1(1:40,2),x1(1:40,3),'r', 'LineWidth', 3.0,'LineSmoothing', 'on');


for i=10:10
    if out(i)==1
        
        scatter3(x1(i,1),x1(i,2),x1(i,3),'r', 'LineWidth', 3.0);
        
        hold on
        scatter3(px(i),py(i),pz(i),10,pz(1), 'filled');
        hold on
        %quiver3(x1(i,1),x1(i,2),x1(i,3),ax(i)/2,ay(i)/2,az(i),'g')
        quiver3(x1(i,1),x1(i,2),x1(i,3),10*cos(psi1(i)*pi/180),10*sin(psi1(i)*pi/180),0,'g')
        hold on
    end
    
end



%scatter3(x1(:,1),x1(:,2),x1(:,4),'r', 'LineWidth', 3.0);
hold on

%quiver3(px',py',pz',nx',ny',nz','g')












