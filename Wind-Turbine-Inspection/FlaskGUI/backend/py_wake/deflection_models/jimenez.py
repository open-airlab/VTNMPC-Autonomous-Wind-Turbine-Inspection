from numpy import newaxis as na
import numpy as np
from backend.py_wake.deflection_models import DeflectionModel


class JimenezWakeDeflection(DeflectionModel):
    """Implemented according to
    Jiménez, Á., Crespo, A. and Migoya, E. (2010), Application of a LES technique to characterize
    the wake deflection of a wind turbine in yaw. Wind Energ., 13: 559-572. doi:10.1002/we.380
    """

    args4deflection = ['D_src_il', 'yaw_ilk', 'ct_ilk', 'tilt_ilk']

    def __init__(self, N=20, beta=.1):
        self.beta = beta
        self.N = N

    def calc_deflection(self, dw_ijl, hcw_ijl, dh_ijl, D_src_il, yaw_ilk, tilt_ilk, ct_ilk, **kwargs):
        dw_lst = (np.logspace(0, 1.1, self.N) - 1) / (10 ** 1.1 - 1)
        dw_ijxl = dw_ijl[:, :, na] * dw_lst[na, na, :, na]
        theta_yaw_ilk, theta_tilt_ilk = np.deg2rad(yaw_ilk), np.deg2rad(-tilt_ilk)
        theta_ilk = np.sqrt(theta_yaw_ilk ** 2 + theta_tilt_ilk ** 2)
        theta_deflection_ilk = np.arctan2(theta_tilt_ilk, theta_yaw_ilk)
        denominator_ilk = np.cos(theta_ilk) ** 2 * np.sin(theta_ilk) * (ct_ilk / 2)
        nominator_ijxl = (1 + (self.beta / D_src_il)[:, na, na, :] * dw_ijxl) ** 2
        alpha = denominator_ilk[:, na, na] / nominator_ijxl[..., na]
        deflection_ijlk = np.trapz(np.sin(alpha), dw_ijxl[..., na], axis=2)
        self.hcw_ijlk = hcw_ijl[..., na] + deflection_ijlk * np.cos(theta_deflection_ilk[:, na])
        self.dh_ijlk = dh_ijl[..., na] + deflection_ijlk * np.sin(theta_deflection_ilk[:, na])
        return dw_ijl[..., na], self.hcw_ijlk, self.dh_ijlk
