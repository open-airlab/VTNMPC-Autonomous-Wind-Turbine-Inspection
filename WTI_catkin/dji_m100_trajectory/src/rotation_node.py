#!/usr/bin/env python

import rospy

import numpy as np
from geometry_msgs.msg import PoseStamped
import numpy as np
from nav_msgs.msg import Odometry

#global theta, psi, phi, p_w,x,y,z



p_w= np.array((-1,0,2))

x=0
y=0
z=0
theta = np.radians(0)
psi=np.radians(0)
phi=np.radians(0)



def pose_cb(data):
    
        #rospy.loginfo(" mocap:")
        
        global theta, psi, phi, p_w, x, y, z
        x = data.pose.position.x
        y = data.pose.position.y
        z = data.pose.position.z
        q0 = data.pose.orientation.w
        q1 = data.pose.orientation.x
        q2 = data.pose.orientation.y
        q3 = data.pose.orientation.z
        
        psi = np.arctan2((2*(q0*q3 + q1*q2)), (1 - 2*(q2**2 + q3**2)) )
        theta = np.arcsin(-2.0*(q0*q3 - q3*q1))
        phi= np.arctan2(2.0*(q0*q1 + q3*q2), q3*q3 + q0*q0 - q1*q1 - q2*q2)
        psi=psi-0.01
        #rospy.loginfo("PSI: %5.3f ",psi*(180/3.142))

def init():

    rospy.init_node('rotation_node')
    rospy.Subscriber("/mavros/mocap/pose", PoseStamped, pose_cb)

def listener():
 
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        x_w=np.array((x,y,z))
        
        R = np.array((  (np.cos(psi)*np.cos(theta)-np.sin(phi)*np.sin(psi)*np.sin(theta), -np.cos(phi)*np.sin(psi), np.cos(psi)*np.sin(theta)+np.cos(theta)*np.sin(phi)*np.sin(psi) ),
                    (np.cos(theta)*np.sin(psi) +np.cos(psi)*np.sin(phi)*np.sin(theta), np.cos(phi)*np.cos(psi), np.sin(psi)*np.sin(theta)-np.cos(psi)*np.cos(theta)*np.sin(phi)),
                    (-np.cos(phi)*np.sin(theta),np.sin(phi),np.cos(phi)*np.cos(theta) )  ))
        b_b = np.array((1,0,0))
        b_w=R.dot(b_b)
        
        n_w=(p_w-x_w)
        #rospy.loginfo("View Error: %5.3f ",n_w[2])
        n_w=n_w/np.linalg.norm(n_w)
        s=b_w[0]*n_w[0]+b_w[1]*n_w[1]+b_w[2]*n_w[2]
        now = rospy.get_rostime()
        rospy.loginfo("View Error: %5.3f ",s)
        rate.sleep()
    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()
    

if __name__ == '__main__':
    init()
    listener()















"""
def calculate():
     
    #rate = rospy.Rate(20)
    #while not rospy.is_shutdown():
        rospy.init_node('rotation_node')
        rospy.Subscriber("/mavros/mocap/pose", Odometry, pose_cb)
        rospy.spin()
        #rospy.init_node('rotation_node')
        #rospy.loginfo("hakim")
        #rospy.Subscriber("/mavros/mocap/pose", Odometry, pose_cb)
        #rospy.loginfo("x: %5.3f ",x)
        #rospy.loginfo("hakim")
        
        x_w=np.array((x,y,z))
        R = np.array((  (np.cos(psi)*np.cos(theta)-np.sin(phi)*np.sin(psi)*np.sin(theta), -np.cos(phi)*np.sin(psi), np.cos(psi)*np.sin(theta)+np.cos(theta)*np.sin(phi)*np.sin(psi) ),
                    (np.cos(theta)*np.sin(psi) +np.cos(psi)*np.sin(phi)*np.sin(theta), np.cos(phi)*np.cos(psi), np.sin(psi)*np.sin(theta)-np.cos(psi)*np.cos(theta)*np.sin(phi)),
                    (-np.cos(phi)*np.sin(theta),np.sin(phi),np.cos(phi)*np.cos(theta) )  ))
        b_b = np.array((1,0,0))
        b_w=R.dot(b_b)
        n_w=(p_w-x_w)
        n_w=n_w/np.linalg.norm(n_w)
        s=b_w[0]*n_w[0]+b_w[1]*n_w[1]+b_w[2]*n_w[2]
        s=-s  
        self=s         
        now = rospy.get_rostime()
        #rospy.loginfo("View Error: %5.3f ",s)
        #rospy.loginfo("x: %5.3f ",x)
        
        rate.sleep()
if __name__ == '__main__':
   #init()
   calculate()
   

"""























"""

def view_error():
    
    rospy.init_node('view_error')
    rospy.Subscriber("/mavros/mocap/pose", Odometry, pose_cb)
    x_w=np.array((x,y,z))
    R = np.array((  (np.cos(psi)*np.cos(theta)-np.sin(phi)*np.sin(psi)*np.sin(theta), -np.cos(phi)*np.sin(psi), np.cos(psi)*np.sin(theta)+np.cos(theta)*np.sin(phi)*np.sin(psi) ),
                  (np.cos(theta)*np.sin(psi) +np.cos(psi)*np.sin(phi)*np.sin(theta), np.cos(phi)*np.cos(psi), np.sin(psi)*np.sin(theta)-np.cos(psi)*np.cos(theta)*np.sin(phi)),
                  (-np.cos(phi)*np.sin(theta),np.sin(phi),np.cos(phi)*np.cos(theta) )  ))
    b_b = np.array((1,0,0))
    b_w=R.dot(b_b)
    n_w=(p_w-x_w)
    n_w=n_w/np.linalg.norm(n_w)
    s=b_w[0]*n_w[0]+b_w[1]*n_w[1]+b_w[2]*n_w[2]
    s=-s
    rospy.loginfo("View Error: %5.3f ",s)
    rospy.loginfo("x: %5.3f ",x)





    rospy.spin()

if __name__ == '__main__':
    view_error()


"""




















