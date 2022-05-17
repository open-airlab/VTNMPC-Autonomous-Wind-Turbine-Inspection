from py_wake.examples.data.hornsrev1 import Hornsrev1Site,V80, wt_x, wt_y, wt16_x, wt16_y
from py_wake import NOJ

windTurbines = V80()
site=Hornsrev1Site()
noj = NOJ(site,windTurbines)



simulationResult = noj(wt16_x,wt16_y)


print ("Total AEP: %f GWh"%simulationResult.aep().sum())
simulationResult.aep()



import matplotlib.pyplot as plt


plt.figure()
aep = simulationResult.aep()
windTurbines.plot(wt16_x,wt16_y)
c =plt.scatter(wt16_x, wt16_y, c=aep.sum(['wd','ws']))
plt.colorbar(c, label='AEP [GWh]')

plt.figure()
aep.sum(['wt','wd']).plot()
plt.xlabel("Wind speed [m/s]")
plt.ylabel("AEP [GWh]")

plt.figure()
aep.sum(['wt','ws']).plot()
plt.xlabel("Wind direction [deg]")
plt.ylabel("AEP [GWh]")



plt.figure()
flow_map = simulationResult.flow_map(ws=10, wd=270)
plt.figure(figsize=(18,10))
flow_map.plot_wake_map()


