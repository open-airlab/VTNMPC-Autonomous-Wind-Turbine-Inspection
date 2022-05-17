#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Oct  8 09:13:31 2021

@author: hakim
"""

import os
import matplotlib.pyplot as plt
import numpy as np
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
print(array2D)

count=1

for i in range(162):
    array2D = np.delete(array2D,count, axis=0)
    count=count+1
    
    
    
    
    