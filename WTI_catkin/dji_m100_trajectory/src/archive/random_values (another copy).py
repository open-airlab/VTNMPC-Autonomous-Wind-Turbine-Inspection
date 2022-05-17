 
 
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

# Import the Python library for ROS
import rospy
# Import the library for generating random numbers
import random
# Import the Twist message from the geometry_msgs package
# Twist data structure is used to represent velocity components
from geometry_msgs.msg import Twist
class RndVelocityGen():
    def __init__(self):


        sites = {"IEA37": IEA37Site(n_wt=16),
         "Hornsrev1": Hornsrev1Site(),
         "ParqueFicticio": ParqueFicticioSite()}
        site = WaspGridSite.from_wasp_grd(ParqueFicticio_path)



# Create a Publisher object, that will publish on /new_vel topic
# messages of type Twist
        self.vel_pub = rospy.Publisher('/new_vel', Twist, queue_size=1)
# Creates var of type Twist
        self.vel = Twist()
# Assign initial velocities
        self.vel.linear.x = 0.5 # m/s
        self.vel.angular.z = 0.5 # rad/s
# Set max values for velocities, min is set to 0
        self.linear_vel_x_max = 1.2
        self.angular_vel_z_max = 0.5
# Set max value for max time interval between velocity changes,
# min is set to 1
        self.max_interval = 10
    def generate_rnd_values(self):
# Loop until someone stops the program execution
        while not rospy.is_shutdown():
            localWinds = {name: site.local_wind(x_i=site.initial_position[4,0], # x position
                                    y_i = site.initial_position[1,1], # y position
                                    h_i=70, # height
                              ws=9, # defaults to 3,4,..,25
                              wd=90, # defaults to 0,1,...,360
                              ) for name, site in sites.items()}
# positive x-vel move the robot forward, negative backward
            V = float(localWinds['ParqueFicticio'].WS)
            Vd= float(localWinds['ParqueFicticio'].WD)
            x_forward=V*np.sin(Vd*3.142/180.0)
# positive z-vel rotate robot counterclockwise, negative clockw
             # z_counterclok = random.choice((-1,1))
            z_counterclok=V*np.cos(Vd*3.142/180.0)
            #
            self.vel.linear.x = V*np.sin(Vd*3.142/180.0)
            self.vel.angular.z = V*np.cos(Vd*3.142/180.0)
            self.vel_pub.publish(self.vel)
            now = rospy.get_rostime()
            print ("Time now: ", now.secs)
            next = (random.randint(1, self.max_interval))
            rospy.loginfo("Twist: [%5.3f, %5.3f], next change in %i secs - ",
            self.vel.linear.x, self.vel.angular.z, next)
            rospy.sleep(next) # Sleeps for the selected seconds
if __name__ == '__main__':
    try:
        generator = RndVelocityGen()
        generator.generate_rnd_values()
    except rospy.ROSInterruptException:
        pass
