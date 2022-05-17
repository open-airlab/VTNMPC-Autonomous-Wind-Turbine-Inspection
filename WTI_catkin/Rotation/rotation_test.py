

import numpy as np

global theta, psi, phi, p_w,x,y,z




    
#def view_error():
p_w= np.array((1,0,0))
x=0
y=0
z=0
theta=np.radians(180)
psi=np.radians(0)
phi=np.radians(0)
    

x_w=np.array((x,y,z))
R = np.array((  (np.cos(psi)*np.cos(theta)-np.sin(phi)*np.sin(psi)*np.sin(theta), -np.cos(phi)*np.sin(psi), np.cos(psi)*np.sin(theta)+np.cos(theta)*np.sin(phi)*np.sin(psi) ),
              (np.cos(theta)*np.sin(psi) +np.cos(psi)*np.sin(phi)*np.sin(theta), np.cos(phi)*np.cos(psi), np.sin(psi)*np.sin(theta)-np.cos(psi)*np.cos(theta)*np.sin(phi)),
              (-np.cos(phi)*np.sin(theta),np.sin(phi),np.cos(phi)*np.cos(theta) )  ))
b_b = np.array((1,0,0))
b_w=R.dot(b_b)
n_w=(p_w-x_w)
n_w=n_w/np.linalg.norm(n_w)
s=b_w[0]*n_w[0]+b_w[1]*n_w[1]+b_w[2]*n_w[2]
print(s)






