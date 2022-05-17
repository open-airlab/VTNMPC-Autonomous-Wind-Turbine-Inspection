close all
clc
clear all

M = dlmread('Results_1/test3.txt');
mx = dlmread('MeshX.txt');
my = dlmread('MeshY.txt');
mz = dlmread('MeshZ.txt');
px =dlmread('px.txt');
py =dlmread('py.txt');
pz =dlmread('pz.txt');


p(1,:)=px;
p(2,:)=py;
p(3,:)=pz;



ts=1550;
M=M(ts:end,:);
x=M(:,5:7);
p=M(:,8:10);
n=M(:,11:13);
x_ref=M(:,2:4);
a= p-x;
a_ref=p-x_ref;
yaw=M(:,19);
yaw=yaw*pi/180;

t=x;


for i=200:201
R=[cos(yaw(i)) -sin(yaw(i)) 0; sin(yaw(i)) cos(yaw(i)) 0; 0 0 1];

t=x(i,:);
point=p(i,:);

  focalLength    = [800, 800]; 
  principalPoint = [320, 240];
  
  imageSize      = [480, 640];
  
  u= x(i,1)/x(i,3);
  v= x(i,1)/x(i,3);
  
  intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize)
  
  
  %imagePoints = worldToImage(intrinsics,R,t,point)
  
end


