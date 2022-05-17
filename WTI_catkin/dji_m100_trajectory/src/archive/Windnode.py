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


def windvelocity():
    
    pub_vx = rospy.Publisher('/px4/cmd_vel', Twist, queue_size=10)
    pub_vy = rospy.Publisher('/px4/cmd_vel', Twist, queue_size=10)
    
    rospy.init_node('wind', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    sites = {"IEA37": IEA37Site(n_wt=16),
         "Hornsrev1": Hornsrev1Site(),
         "ParqueFicticio": ParqueFicticioSite()}
    site = WaspGridSite.from_wasp_grd(ParqueFicticio_path)
    
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
       # rospy.loginfo(hello_str)
        localWinds = {name: site.local_wind(x_i=site.initial_position[4,0], # x position
                                    y_i = site.initial_position[1,1], # y position
                                    h_i=70, # height
                              ws=9, # defaults to 3,4,..,25
                              wd=90, # defaults to 0,1,...,360
                              ) for name, site in sites.items()}
        V=float(localWinds['ParqueFicticio'].WS)
        Vd=float(localWinds['ParqueFicticio'].WD)
        Vy=V*np.cos(Vd*3.142/180.0)
        Vx=V*np.sin(Vd*3.142/180.0)        
        pub_vx.publish(Vx)
        pub_vy.publish(Vxy)
        rate.sleep()
    
if __name__ == '__main__':
    try:
        windvelocity()
    except rospy.ROSInterruptException:
        pass