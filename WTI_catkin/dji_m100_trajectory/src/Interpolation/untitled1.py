#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Oct 17 11:34:52 2021

@author: hakim
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Oct  8 09:00:36 2021

@author: hakim
"""
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt
import numpy as np
import os


d = "/home/hakim/catkin_ws/src/WTI_catkin/dji_m100_trajectory/src/Interpolation"


array2D = []

filename="px.txt"

with open(filename, 'r') as f:
     for line in f.readlines():
            array2D.append(line.split(' '))
            
a=array2D

d = "/home/hakim/catkin_ws/src/WTI_catkin/dji_m100_trajectory/src/Interpolation"


array2D1 = []

filename="py.txt"

with open(filename, 'r') as f:
     for line in f.readlines():
            array2D1.append(line.split(' '))


b=array2D1

d = "/home/hakim/catkin_ws/src/WTI_catkin/dji_m100_trajectory/src/Interpolation"




b=array2D1

d = "/home/hakim/catkin_ws/src/WTI_catkin/dji_m100_trajectory/src/Interpolation"


array2D2 = []

filename="pz.txt"

with open(filename, 'r') as f:
     for line in f.readlines():
            array2D2.append(line.split(' '))



c=array2D2






array2D3 = []




#array2D = np.delete(array2D, 4, 1) 
#array2D = np.delete(array2D, 4, 1) 
#array2D = np.delete(array2D, 3, 1)
#data=np.column_stack((a,b,c))


#count=1
c = np.array(c, dtype=np.float32)
a = np.array(a, dtype=np.float32)
b = np.array(b, dtype=np.float32)


for i in range(80):
    c[2*i+1] = c[2*i+1]
    c[2*i+2] = c[2*i+2]-0.000009
    
    a[2*i+1] = a[2*i+1]
    a[2*i+2] = a[2*i+2]-0.000004
    
    b[2*i+1] = b[2*i+1]
    b[2*i+2] = b[2*i+2]+0.000005
   
c[0]=c[0]+0.0001
a[0]=a[0]+0.001
b[0]=b[0]+0.00001

#data=array2D

                
from scipy import interpolate
from mpl_toolkits.mplot3d import Axes3D

data = np.array(data, dtype=np.float32)

data = data.astype(float)
data = data.transpose()
#data = data.transpose()
a=a.transpose()
b=b.transpose()
c=c.transpose()
#now we get all the knots and info about the interpolated spline
tck, u= interpolate.splprep(a, k=1)
#here we generate the new interpolated dataset, 
#increase the resolution by increasing the spacing, 500 in this example
new = interpolate.splev(np.linspace(0,1,5000), tck, der=0)

numpy_array = np. array(new)
transpose = numpy_array.T
out = transpose.tolist()


np.savetxt('/home/hakim/catkin_ws/src/WTI_catkin/dji_m100_trajectory/src/Interpolation/inter.txt', out, fmt = '%.6f')

#now lets plot it!
fig = plt.figure()
ax = Axes3D(fig)
ax.grid(False)
ax.set_facecolor('white')
ax.plot(data[0], data[1], data[2], label='Original Global Path', lw =2, c='blue')
ax.plot(new[0], new[1], new[2], label='Interpolated Trajectory', lw =2, c='black')
ax.legend()
plt.savefig('junk.png')
plt.show()