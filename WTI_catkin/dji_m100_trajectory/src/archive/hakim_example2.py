try:
    import py_wake
except ModuleNotFoundError:
    !pip install git+https://gitlab.windenergy.dtu.dk/TOPFARM/PyWake.git
    
# import and setup site and windTurbines
import numpy as np
import matplotlib.pyplot as plt
from py_wake.examples.data.iea37 import IEA37Site, IEA37_WindTurbines
from py_wake import IEA37SimpleBastankhahGaussian


from py_wake.site import XRSite
from py_wake.utils import weibull
import xarray as xr

from py_wake.examples.data.hornsrev1 import Hornsrev1Site
from py_wake.examples.data.iea37 import IEA37Site
from py_wake.examples.data.ParqueFicticio import ParqueFicticioSite

f = [0.036, 0.039, 0.052, 0.07, 0.084, 0.064, 0.086, 0.118, 0.152, 0.147, 0.1, 0.052]
#A = [9.177, 9.782, 9.532, 9.91, 10.043, 9.594, 9.584, 10.515, 11.399, 11.687, 11.637, 10.088]
#k = [2.393, 2.447, 2.412, 2.592, 2.756, 2.596, 2.584, 2.549, 2.471, 2.607, 2.627, 2.326]
wd = np.linspace(0, 360, len(f), endpoint=False)
#ti = .1



site = IEA37Site(16)
x, y = site.initial_position.T
windTurbines = IEA37_WindTurbines()

wf_model = IEA37SimpleBastankhahGaussian(site, windTurbines)

print(wf_model)
# run wind farm simulation
sim_res = wf_model(x, y, # wind turbine positions
                   h=None, # wind turbine heights (defaults to the heights defined in windTurbines)
                   type=0, # Wind turbine types
                   wd=None, # Wind direction (defaults to site.default_wd (0,1,...,360 if not overriden))
                   ws=None, # Wind speed (defaults to site.default_ws (3,4,...,25m/s if not overriden))
                  )

flow_map = sim_res.flow_map(grid=None, # defaults to HorizontalGrid(resolution=500, extend=0.2), see below
                            wd=0,
                            ws=None)


complex_grid_site = XRSite(
    ds=xr.Dataset(
        data_vars={'Speedup': (['x', 'y'], np.arange(.8, 1.4, .1).reshape((3, 2))),
                   'P': ('wd', f)},
        coords={'x': [0, 500, 1000], 'y': [0, 500], 'wd': wd}))
#flow_map.plot_wake_map()


flow_map.plot_wake_map()
#flow_map.plot_ti_map()
#flow_map.plot_windturbines


sites = {"IEA37": IEA37Site(n_wt=16),
         "Hornsrev1": Hornsrev1Site(),
         "ParqueFicticio": ParqueFicticioSite()}
from py_wake.site import UniformWeibullSite
site = UniformWeibullSite(p_wd = [.20,.25,.35,.25], # sector frequencies
                          a = [9.176929,  9.782334,  9.531809,  9.909545], # Weibull scale parameter
                          k = [2.392578, 2.447266, 2.412109, 2.591797], # Weibull shape parameter
                          ti = 0.1 # turbulence intensity, optional (not needed in all cases)
                         )



localWinds = {name: site.local_wind(x_i=site.initial_position[:,0], # x position
                                    y_i = site.initial_position[:,1], # y position
                                    h_i=site.initial_position[:,0]*0+70, # height
                              ws=None, # defaults to 3,4,..,25
                              wd=None, # defaults to 0,1,...,360
                              ) for name, site in sites.items()}
localWinds['IEA37']

xr.LocalWind