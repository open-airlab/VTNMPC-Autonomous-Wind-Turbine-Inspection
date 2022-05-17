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


d = "/home/hakim/catkin_ws/src/WTI_catkin/test_scripts"


array2D = []

for filename in os.listdir(d):
    if not filename.endswith('.txt'):
        continue

    with open(filename, 'r') as f:
        for line in f.readlines():
            array2D.append(line.split(' '))

array2D = np.delete(array2D, 4, 1) 
array2D = np.delete(array2D, 4, 1) 
array2D = np.delete(array2D, 3, 1)


count=1

for i in range(162):
    array2D = np.delete(array2D,count, axis=0)
    count=count+1


data=array2D

                
from scipy import interpolate
from mpl_toolkits.mplot3d import Axes3D

data = data.astype(float)
data = data.transpose()


#now we get all the knots and info about the interpolated spline
tck, u= interpolate.splprep(data, k=5)
#here we generate the new interpolated dataset, 
#increase the resolution by increasing the spacing, 500 in this example
new = interpolate.splev(np.linspace(0,1,5000), tck, der=0)

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
