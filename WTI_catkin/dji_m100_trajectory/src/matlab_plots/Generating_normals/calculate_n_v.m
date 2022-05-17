%% inspectionPathVisualization.m
% 
% This script was written by Andreas Bircher on 6th October 2014
%         andreas.bircher@mavt.ethz.ch / bircher@gmx.ch
% 
% Scenarios for autonomous inspection are visualized, for which the path is
% computed using the provided planner.
% 
%% 
% this->a = ((this->x2-this->x1).cross(this->x3-this->x2))/2;
clear all
clc

mx = dlmread('MeshX.txt');
my = dlmread('MeshY.txt');
mz = dlmread('MeshZ.txt');
path=dlmread('path.txt');


for i=1:length(path)/2
   path(i+1,:)=[]; 
end
path(:,4:6)=[];
path_i=path;
px=(mx(:,1)+mx(:,2)+mx(:,3))/3;
py=(my(:,1)+my(:,2)+my(:,3))/3;
pz=(mz(:,1)+mz(:,2)+mz(:,3))/3;

v1=[mx(:,1) my(:,1) mz(:,1)];
v2=[mx(:,2) my(:,2) mz(:,2)];
v3=[mx(:,3) my(:,3) mz(:,3)];


v=v2-v1;
vx=v(:,1);
vy=v(:,2);
vz=v(:,3);


for i=1:length(mx)
    
        n(i,:)=cross((v2(i,:)-v1(i,:)),(v3(i,:)-v2(i,:)))/2;
        n(i,:)=n(i,:)/(n(i,1)^2+n(i,2)^2+n(i,3)^2)^0.5;
        %cross((mx(i,2)-mx(i,1)),(mx(i,3)-mx(i,2)))/2;
       % ax(i)=cross((mx(i,2)-mx(i,1)),(mx(i,3)-mx(i,2)))/2;
      %  ay(i)=cross((my(i,2)-my(i,1)),(my(i,3)-my(i,2))/2;
       % az(i)=cross((mz(i,2)-mz(i,1)),(mz(i,3)-mz(i,2)))/2;
        
    
end
nx=n(:,1);
ny=n(:,2);
nz=n(:,3);
d=1000;

for i=1:length(path)
for j=1:length(path)
    
    d1 = ((path(i,1)-px(j))^2+(path(i,2)-py(j))^2+(path(i,3)-pz(j))^2)^0.5;
    
    %d1= (Path(2*i-1,2)-y(j))^2;
    %d1=(path_new(i,2)-py(j))^2;
    
    if (d1-0)^2<(d-0)^2
        d=d1;
        n=j;
        
    end
    

end 




nn(i)=n;

dd(i)=d;
x1(i)=px(n);
y1(i)=py(n);
z1(i)=pz(n);
nx_o(i)=nx(n);
ny_o(i)=ny(n);
nz_o(i)=nz(n);
vx_o(i)=vx(n);
vy_o(i)=vy(n);
vz_o(i)=vz(n);
d=100000;
n=0;
end


px=x1;
py=y1;
pz=z1;
nx=nx_o;
ny=ny_o;
nz=nz_o;

vx=vx_o;
vy=vy_o;
vz=vz_o;


%v=v_o;




for j=1:1
for  i=1:length(px)-1
    px1(2*i-1)=px(i);
    px1(2*i)=(px(i)+px(i+1))/2;
    
    px1(2*i+1)=px(i+1);
    
    py1(2*i-1)=py(i);
    py1(2*i)=(py(i)+py(i+1))/2;
    
    py1(2*i+1)=py(i+1);
    
    pz1(2*i-1)=pz(i);
    pz1(2*i)=(pz(i)+pz(i+1))/2;
    
    pz1(2*i+1)=pz(i+1);
    
    
    nx1(2*i-1)=nx(i);
    nx1(2*i)=(nx(i)+nx(i+1))/2;
    
    nx1(2*i+1)=nx(i+1);
    
    ny1(2*i-1)=ny(i);
    ny1(2*i)=(ny(i)+ny(i+1))/2;
    
    ny1(2*i+1)=ny(i+1);
    
    nz1(2*i-1)=nz(i);
    nz1(2*i)=(nz(i)+nz(i+1))/2;
    
    nz1(2*i+1)=nz(i+1);
    
    

    
    vx1(2*i-1)=vx(i);
    vx1(2*i)=(vx(i)+vx(i+1))/2;
    vx1(2*i+1)=vx(i+1);
    
    vy1(2*i-1)=vy(i);
    vy1(2*i)=(vy(i)+vy(i+1))/2;
    vy1(2*i+1)=vy(i+1);
    
    vz1(2*i-1)=vz(i);
    vz1(2*i)=(vz(i)+vz(i+1))/2;
    vz1(2*i+1)=vz(i+1);
    
    
    
    path1(2*i-1,:)= path(i,:);
    path1(2*i,:)=( path(i,:)+ path(i+1,:))/2;
    path1(2*i+1,:)= path(i+1,:);
    
    
    
    if i==length(px)-1
        path=path1;
        px=px1;
        py=py1;
        pz=pz1;
        nx=nx1;
        ny=ny1;
        nz=nz1;
        vx=vx1;
        vy=vy1;
        vz=vz1;
        
    end
    
 
end
end


writematrix(px', "px.txt");
writematrix(py', "py.txt");
writematrix(pz', "pz.txt");

writematrix(nx', "nx.txt");
writematrix(ny', "ny.txt");
writematrix(nz', "nz.txt");

writematrix(vx', "vx.txt");
writematrix(vy', "vy.txt");
writematrix(vz', "vz.txt");
writematrix(v, "v.txt");
writematrix(n, "n.txt");







%run('inspectionScenario');
set(0,'defaultfigurecolor',[1 1 1])
handle = figure;
plot3(path(:,1),path(:,2),path(:,3),'b', 'LineWidth', 2.0,'LineSmoothing', 'on');
hold on
quiver3(px',py',pz',nx',ny',nz','g')
hold on;
patch(mx',my',mz','r','EdgeColor','k','FaceAlpha',0.7);
xlabel('x[m]');
ylabel('y[m]');
zlabel('z[m]');

legend('Optimal Path','Inspected Wind Turbine Blades')
title(['Global Planner Inspection Path ']);
axis equal;

