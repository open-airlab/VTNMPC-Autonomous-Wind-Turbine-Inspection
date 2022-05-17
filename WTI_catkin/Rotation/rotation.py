        #!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
# Import the Python library for ROS
import rospy
# Import the library for generating random numbers
import random
# Import the Twist message from the geometry_msgs package
# Twist data structure is used to represent velocity components
from geometry_msgs.msg import Twist
import numpy as np
from nav_msgs.msg import Odometry

global theta, psi, phi, p_w,x,y,z
#theta = np.radians(0)
#psi=np.radians(0)
#phi=np.radians(0)


p_w= np.array((1,1,0))
"""
R = np.array((  (np.cos(psi)*np.cos(theta)-np.sin(phi)*np.sin(psi)*np.sin(theta), -np.cos(phi)*np.sin(psi), np.cos(psi)*np.sin(theta)+np.cos(theta)*np.sin(phi)*np.sin(psi) ),
(np.cos(theta)*np.sin(psi) +np.cos(psi)*np.sin(phi)*np.sin(theta), np.cos(phi)*np.cos(psi), np.sin(psi)*np.sin(theta)-np.cos(psi)*np.cos(theta)*np.sin(phi)),
(-np.cos(phi)*np.sin(theta),np.sin(phi),np.cos(phi)*np.cos(theta) )  ))



print('rotation matrix:')
print(R)

v = np.array((1,1,1))

print('vector v: ')
print(v)

print('apply the rotation matrix r to v: r*v')
print( R.dot(v) )
"""


def pose_cb(data):
        global psi, theta, phi,x,y,z
        x = data.pose.position.x
        y = data.pose.position.y
        z = data.pose.position.z
        q0 = data.pose.orientation.w;
        q1 = data.pose.orientation.w;
        q2 = data.pose.orientation.w;
        q3 = data.pose.orientation.w;
        psi = np.atan2((2*(q0*q3 + q1*q2)), (1 - 2*(q2^2 + q3^2)) );
        theta = np.asin(-2.0*(q0*q3 - q3*q1));
        phi= np.atan2(2.0*(q0*q1 + q3*q2), q3*q3 + q0*q0 - q1*q1 - q2*q2);

    
def view_error():

    
    rospy.init_node('view_error')
    rospy.Subscriber("/mavros/mocap/pose", Odometry, pose_cb)
    x_w=np.array((x,y,z))
    R = np.array((  (np.cos(psi)*np.cos(theta)-np.sin(phi)*np.sin(psi)*np.sin(theta), -np.cos(phi)*np.sin(psi), np.cos(psi)*np.sin(theta)+np.cos(theta)*np.sin(phi)*np.sin(psi) ),
                  (np.cos(theta)*np.sin(psi) +np.cos(psi)*np.sin(phi)*np.sin(theta), np.cos(phi)*np.cos(psi), np.sin(psi)*np.sin(theta)-np.cos(psi)*np.cos(theta)*np.sin(phi)),
                  (-np.cos(phi)*np.sin(theta),np.sin(phi),np.cos(phi)*np.cos(theta) )  ))
    b_b = np.array((1,0,0))
    b_w=R.dot(b_b)
    n_w=p_w-x_w
    s=b_w[0]*n_w[0]+b_w[1]*n_w[1]+b_w[2]*n_w[2]
    print(s)




    rospy.spin()

if __name__ == '__main__':
    view_error()





















