import os
import time
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
import xarray as xr
import yaml

from backend.py_wake import IEA37SimpleBastankhahGaussian, HorizontalGrid
from backend.py_wake.deficit_models import ZongGaussianDeficit, SelfSimilarityDeficit2020
from backend.py_wake.deflection_models import JimenezWakeDeflection
from backend.py_wake.ground_models import Mirror
from backend.py_wake.rotor_avg_models import CGIRotorAvg
from backend.py_wake.site import XRSite
from backend.py_wake.site.shear import PowerShear
from backend.py_wake.superposition_models import LinearSum
from backend.py_wake.turbulence_models import STF2017TurbulenceModel
from backend.py_wake.wind_farm_models import All2AllIterative
from backend.py_wake.wind_turbines import WindTurbines
from backend.py_wake.wind_turbines.generic_wind_turbines import GenericWindTurbine, GenericTIRhoWindTurbine

wind_profile = Path(os.path.dirname(__file__), "configs", "wind_profile_config.yml")


class WindFarm:
    turb_time = 0
    turb_direction_vector = np.array([1, 0, 0])
    turb_magnitude = 0
    rate = 100  # ms
    scalar = 0.28  # 1 is high, 0 is none

    def __init__(self, wind_speed=0.1, wind_direction=0, debug_mode=False):
        # Load profile:
        self.config = {}
        with open(str(wind_profile), "r") as f:
            self.config = yaml.safe_load(f)
        self.windspeed_lower = self.load_config("windspeed_lower", fallback=0.1)
        self.windspeed_upper = self.load_config("windspeed_upper", fallback=29.6)
        self.turb_mag_limit = self.load_config("turbulence_magnitude", fallback=5)

        self.wts_list = []
        self.wf_model = None
        self.uniform_site = None
        self.wind_speed = wind_speed if self.windspeed_lower < wind_speed < self.windspeed_upper else self.windspeed_lower
        self.wind_direction = wind_direction % 360
        self._x = []
        self._y = []
        self._h = []
        self.flow_box = None
        self.flow_map = None
        self.change_happen_since_last_compile = False
        self.DEBUG_MODE = debug_mode

    def load_config(self, name, fallback):
        return self.config[name] if name in self.config else fallback

    def set_wind_speed(self, ws):
        if ws < self.windspeed_lower:
            self.wind_speed = self.windspeed_lower
        elif ws > self.windspeed_upper:
            self.wind_speed = self.windspeed_upper
        else:
            self.wind_speed = ws

    def set_wind_direction(self, wd):
        self.wind_direction = wd

    def add_wind_turbine(self, name, type_name, diameter, hub_height, xy_pos, power_norm=10000,
                         turbulence_intensity=.1):
        wt = MyWindTurbine2(name, type_name=type_name, diameter=diameter, hub_height=hub_height, xy_pos=xy_pos,
                            power_norm=power_norm,
                            turbulence_intensity=turbulence_intensity)
        self.wts_list.append(wt)
        self.change_happen_since_last_compile = True

    def get_wind_turbines(self):
        return self.wts_list

    def compile_wind_farm_model(self):
        _wts = []
        self._x = []
        self._y = []
        self._h = []
        for turbine in self.wts_list:
            _wts.append(turbine.wt)
            self._x.append(turbine.x)
            self._y.append(turbine.y)
            self._h.append(turbine.height)
        wts = WindTurbines.from_WindTurbine_lst(_wts)
        turbulence_intensity = .1

        # Site with constant wind speed, sector frequency, constant turbulence intensity and power shear
        self.uniform_site = XRSite(initial_position=zip(self._x, self._y),
                                   ds=xr.Dataset(data_vars={'P': ('wd', [1.]), 'TI': turbulence_intensity},
                                                 coords={'wd': [self.wind_direction]}),
                                   shear=PowerShear(h_ref=100, alpha=.2))

        if self.DEBUG_MODE:
            # This is much faster version.
            self.wf_model = IEA37SimpleBastankhahGaussian(self.uniform_site, wts,
                                                          turbulenceModel=STF2017TurbulenceModel())
        else:
            self.wf_model = All2AllIterative(site=self.uniform_site,
                                             windTurbines=wts,
                                             wake_deficitModel=ZongGaussianDeficit(a=[0.38, 4e-3]),
                                             rotorAvgModel=CGIRotorAvg(n=21),
                                             superpositionModel=LinearSum(),
                                             blockage_deficitModel=SelfSimilarityDeficit2020(),
                                             deflectionModel=JimenezWakeDeflection(),
                                             turbulenceModel=STF2017TurbulenceModel(),
                                             groundModel=Mirror())
        # self.wf_model = IEA37SimpleBastankhahGaussian(site=self.uniform_site,
        #                                               windTurbines=wts,
        #                                               rotorAvgModel=CGIRotorAvg(n=21),
        #                                               deflectionModel=JimenezWakeDeflection(),
        #                                               turbulenceModel=STF2017TurbulenceModel(),
        #                                               groundModel=Mirror())
        test_wind = self.uniform_site.local_wind(400, 400, 100)
        self.change_happen_since_last_compile = False

    def generate_flow_map(self, wind_speed=None, wind_direction=None, map_size=800, gen_flow_box=True):
        """
        :param wind_speed: defaults back to class variable
        :param wind_direction: defaults back to class variable
        :param map_size: size of output image.
        :return:
        """
        if wind_speed is None or not (self.windspeed_lower < wind_speed < self.windspeed_upper):
            wind_speed = self.wind_speed
        if wind_direction is None:
            wind_direction = self.wind_direction
        if self.change_happen_since_last_compile:
            self.compile_wind_farm_model()
        sim_res = self.wf_model.__call__(x=self._x, y=self._y, wd=wind_direction, ws=wind_speed, verbose=True)
        if not gen_flow_box:
            ext = 0.12
            self.flow_map = sim_res.flow_map(grid=HorizontalGrid(resolution=map_size, extend=ext),
                                             # defaults to HorizontalGrid(resolution=500, extend=0.2), see below
                                             wd=wind_direction,
                                             ws=wind_speed)

            return self.flow_map
        else:
            ext = 50
            self.flow_box = sim_res.flow_box(
                x=np.linspace(min(self._x) - ext, max(self._x) + ext, 801),
                y=np.linspace(min(self._y) - ext, max(self._y) + ext, 801),
                h=np.linspace(25, max(self._h) + 25, 10))
            debug = 0
            return self.flow_box,

    def get_wind_at_pos(self, x, y, z=None, use_2d_mode=False):
        if self.flow_box is None and not use_2d_mode:
            print("Need to compute flow box, click the generate flow map button.")
            return False
        if not use_2d_mode:
            res = self.flow_box.WS_eff.sel(x=x, y=y, h=z, method="nearest")
        else:
            res = self.flow_map.WS_eff.sel(x=x, y=y, method="nearest")
        wd = res.wd.data
        ws = res.values
        return ws, wd

    @staticmethod
    def time_ms():
        return time.time() * 1000

    @staticmethod
    def get_step_size(TI, scalar):
        """
        Compute variance in gaussian distribution
        :param TI: Turbulence intensity
        :return:
        """

        variance = TI * scalar
        step_size = variance
        return step_size

    def plot3D(self, x, y, z, old_x, old_y, old_z):
        # Creating an empty canvas(figure)
        fig = plt.figure()
        # Using the gca function, we are defining
        # the current axes as a 3D projection
        ax = fig.add_subplot(111, projection='3d')
        # Labelling X-Axis
        ax.set_xlabel('longitude-Axis')
        # Labelling Y-Axis
        ax.set_ylabel('latitude-Axis')
        # Labelling Z-Axis
        ax.set_zlabel('Z-Axis')
        ax.set_xlim(-self.turb_mag_limit, self.turb_mag_limit)

        ax.set_ylim(-self.turb_mag_limit, self.turb_mag_limit)

        ax.set_zlim(-self.turb_mag_limit, self.turb_mag_limit)

        # Plot point:
        # ax.scatter(x, y, z, c='r')
        # ax.scatter(old_x, old_y, old_z, c='b')
        ax.quiver(0, 0, 0, x, y, z, color='r')
        ax.quiver(0, 0, 0, old_x, old_y, old_z, color='b')
        # plt.show()
        img = self._convert_to_cv_img(fig)
        plt.clf()
        return img

    def get_turbulence_at_pos(self, x, y, z, use_2d_mode=False):
        # Compute direction:
        if not use_2d_mode:
            TI = self.flow_box.TI_eff.sel(x=x, y=y, h=z, method="nearest")
        else:
            TI = self.flow_map.TI_eff.sel(x=x, y=y, method="nearest")
        ti = np.squeeze(TI.values)
        step_size = self.get_step_size(ti, self.scalar)
        turb_direction = self.turb_direction_vector
        new_time_ms = self.time_ms()
        old_turb_mag = self.turb_magnitude

        epoch = min((new_time_ms - self.turb_time) / self.rate, 20)  # Lets not do more than 20 epochs to catch up
        for i in range(max(int(np.ceil(epoch)), 1)):
            new_orientation = np.random.normal(turb_direction, step_size)
            new_orientation = new_orientation / np.linalg.norm(new_orientation)
            turb_direction = new_orientation

            new_turb_mag = np.random.normal(old_turb_mag, step_size)
            if new_turb_mag < 0:
                new_turb_mag = abs(new_turb_mag)
            if new_turb_mag > self.turb_mag_limit:
                new_turb_mag = new_turb_mag - (new_turb_mag - self.turb_mag_limit)
            new_turb_mag = np.clip(new_turb_mag, 0, self.turb_mag_limit)
            old_turb_mag = new_turb_mag

        plot_new_vector = new_orientation * new_turb_mag
        plot_old_vector = self.turb_direction_vector * self.turb_magnitude

        # TODO:: Compute magnitude:
        self.turb_direction_vector = new_orientation
        self.turb_magnitude = new_turb_mag
        self.turb_time = new_time_ms

        img = self.plot3D(plot_new_vector[0], plot_new_vector[1], plot_new_vector[2],
                          plot_old_vector[0], plot_old_vector[1], plot_old_vector[2])

        return self.turb_direction_vector * self.turb_magnitude,self.turb_magnitude, img, float(ti)

    @staticmethod
    def apply_str(wind_vec, str):
        norm1 = wind_vec / np.sum(np.abs(wind_vec))
        return norm1 * str

    @staticmethod
    def cart2pol(x, y):
        rho = np.sqrt(x ** 2 + y ** 2)
        phi = np.arctan2(y, x)
        return (rho, phi)

    @staticmethod
    def pol2cart(rho, phi):
        x = rho * np.cos(phi)
        y = rho * np.sin(phi)
        return (x, y)

    def get_pixel_index(self, x, y):
        if self.flow_map is None:
            self.generate_flow_map(gen_flow_box=False)
        value, x_nearest = self.find_nearest(self.flow_map.x.values, x)
        value, y_nearest = self.find_nearest(self.flow_map.y.values, y)
        return x_nearest, y_nearest

    @staticmethod
    def find_nearest(array, value):
        array = np.asarray(array)
        idx = (np.abs(array - value)).argmin()
        return array[idx], idx

    @staticmethod
    def _convert_to_cv_img(fig):
        fig.canvas.draw()
        img = np.frombuffer(fig.canvas.tostring_rgb(), dtype='uint8')
        # img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8,
        #                     sep='')
        img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img


class MyWindTurbine:
    def __init__(self, name, type_name, diameter, hub_height, xy_pos, power_norm=10000, turbulence_intensity=.1):
        self.name = name
        self.wt = GenericWindTurbine(name=type_name, diameter=diameter, hub_height=hub_height, power_norm=power_norm,
                                     turbulence_intensity=turbulence_intensity)
        self.x = xy_pos[0]
        self.y = xy_pos[1]
        self.height = hub_height


class MyWindTurbine2(MyWindTurbine):
    def __init__(self, name, type_name, diameter, hub_height, xy_pos, power_norm=10000, turbulence_intensity=.1):
        super().__init__(name, type_name, diameter, hub_height, xy_pos, power_norm, turbulence_intensity)
        self.wt = GenericTIRhoWindTurbine(type_name, diameter, hub_height, power_norm=power_norm,
                                          TI_eff_lst=np.linspace(0, .5, 6), default_TI_eff=.1,
                                          Air_density_lst=np.linspace(.9, 1.5, 5), default_Air_density=1.225)


if __name__ == '__main__':
    # turbulence intensity = TI
    test_py_wake = False
    if test_py_wake:
        gen_wt = GenericWindTurbine(name='G10MW', diameter=178.3, hub_height=119, power_norm=10000,
                                    turbulence_intensity=.1)
        wt = GenericTIRhoWindTurbine('2MW', 80, 70, power_norm=2000,
                                     TI_eff_lst=np.linspace(0, .5, 6), default_TI_eff=.1,
                                     Air_density_lst=np.linspace(.9, 1.5, 5), default_Air_density=1.225)

        wts = WindTurbines.from_WindTurbine_lst([gen_wt, gen_wt])
        position = [[1000, 1000], [1500, 1500]]
        # wts.plot([1000, 1500], [1000, 1500])
        # plt.show()

        # Example:
        f = [0.036, 0.039, 0.052, 0.07, 0.084, 0.064, 0.086, 0.118, 0.152, 0.147, 0.1, 0.052]
        wd = np.linspace(0, 360, len(f), endpoint=False)
        ti = .1
        # Site with constant wind speed, sector frequency, constant turbulence intensity and power shear
        uniform_site = XRSite(
            ds=xr.Dataset(data_vars={'WS': 10, 'P': ('wd', f), 'TI': ti},
                          coords={'wd': wd}),
            initial_position=position,
            shear=PowerShear(h_ref=100, alpha=.2))

        wf_model = IEA37SimpleBastankhahGaussian(uniform_site, wts)
        sim_res = wf_model.__call__(x=[1000, 1500], y=[1000, 1500], wd=0, ws=10, verbose=True)
        flow_map = sim_res.flow_map(grid=HorizontalGrid(resolution=800, extend=0.2),
                                    # defaults to HorizontalGrid(resolution=500, extend=0.2), see below
                                    wd=0,
                                    ws=10)
        flow_map.plot_wake_map()
        plt.show()
    else:
        wf = WindFarm(wind_speed=10)
        wf.add_wind_turbine(type_name="SimTurbine", name="1", diameter=178.3, hub_height=119, xy_pos=[2000, 1000],
                            power_norm=10000,
                            turbulence_intensity=.1)

        wf.add_wind_turbine(type_name="SimTurbine", name="2", diameter=178.3, hub_height=119, xy_pos=[2000, 2000],
                            power_norm=10000,
                            turbulence_intensity=.1)

        wf.add_wind_turbine(type_name="SimTurbine", name="3", diameter=178.3, hub_height=119, xy_pos=[3000, 2000],
                            power_norm=10000,
                            turbulence_intensity=.1)
        for i in range(0, 360, 10):
            wf.set_wind_direction(i)
            img = wf.generate_flow_map()
            cv2.imshow("plot", img)
            cv2.waitKey(10)
        cv2.destroyAllWindows()
