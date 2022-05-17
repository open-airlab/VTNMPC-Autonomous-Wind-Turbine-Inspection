#! /usr/bin/env python3 
 
from py_wake.examples.data.hornsrev1 import Hornsrev1Site
from py_wake.examples.data.iea37 import IEA37Site
from py_wake.examples.data.ParqueFicticio import ParqueFicticioSite
import numpy as np
import matplotlib.pyplot as plt
from py_wake.site import WaspGridSite
from py_wake.examples.data.ParqueFicticio import ParqueFicticio_path
from py_wake.site import XRSite
from py_wake.site.shear import PowerShear
import xarray as xr
import numpy as np
from py_wake.utils import weibull
from numpy import newaxis as na
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

x_d = 0.0
y_d = 0.0
z_d = 0
yaw_d = 0
wd=0
global xt, yt
#xt={1, 2,3}
#yt={1, 2,3}
def pose_cb(data):
        global x_d, y_d
        x_d = data.pose.position.x
        y_d = data.pose.position.y
class RndVelocityGen():

    

    def __init__(self):

        rospy.init_node('random_velocity')
        self.vel_pub = rospy.Publisher('/new_vel', Twist, queue_size=1)
        rospy.Subscriber("/mavros/mocap/pose", Odometry, pose_cb)
        self.vel = Twist()

    def generate_rnd_values(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            sites = {"IEA37": IEA37Site(n_wt=16),
            "Hornsrev1": Hornsrev1Site(),
            "ParqueFicticio": ParqueFicticioSite()}
            site = WaspGridSite.from_wasp_grd(ParqueFicticio_path)


            localWinds = {name: site.local_wind(262878+x_d*100, # x position
                                    y_i = 6504714+y_d*100, # y position
                                    h_i=70, # height
                              ws=4, # defaults to 3,4,..,25
                              wd=90, # defaults to 0,1,...,360
                              ) for name, site in sites.items()}
            V = float(localWinds['ParqueFicticio'].WS)
            Vd= float(localWinds['ParqueFicticio'].WD)
            T=float(localWinds['ParqueFicticio'].TI)
            t_rx = (random.randint(-1,1))
            t_ry= (random.randint(-1,1))

            Vx=V*np.cos(Vd*3.142/180.0)
            Vy=V*np.sin(Vd*3.142/180.0)
            xt=T*Vx*t_rx
            yt=T*Vy*t_ry
            self.vel.linear.x = Vx+xt
            self.vel.linear.y = Vy+yt
            self.vel.angular.z = V*np.cos(Vd*3.142/180.0)
            self.vel_pub.publish(self.vel)
            now = rospy.get_rostime()
            rospy.loginfo("Wind x and y velocities: [%5.3f, %5.3f] ",
            self.vel.linear.x, self.vel.linear.y)
            rate.sleep()
if __name__ == '__main__':
    try:
        generator = RndVelocityGen()
        generator.generate_rnd_values()
    except rospy.ROSInterruptException:
        pass
