import gurobipy as gp
from gurobipy import GRB
import numpy as np
import time
from . BlimpController import BlimpController
from . CtrlCBF import CtrlCBF
from . Trajectories import Trajectories
from . utilities import *
import sys

class CtrlCBFLine(CtrlCBF):

    def __init__(self, dT):
        super().__init__(dT)
        
        self.metadata = np.array([
            f"gamma_th = {self.gamma_th}",
            f"gamma_ph = {self.gamma_ph}",
            f"gamma_psi = {self.gamma_ps}",
            f"k1 = {self.k1.T}",
            f"k2 = {self.k2.T}",
            f"theta_limit = {self.theta_limit}",
            f"phi_limit = {self.phi_limit}",
            f"psi_limit = {self.psi_limit}",
            f"psi cbf = {self.use_psi_cbf}",
            f"dT = {dT}"
        ])
        
    def init_sim(self, sim):
        x0 = sim.get_var('x')
        y0 = sim.get_var('y')
        z0 = sim.get_var('z')
        psi0 = sim.get_var('psi')
        
        trajectory = Trajectories.get_line(x0, y0, z0, psi0, self.dT)
        self.init_trajectory(trajectory)
        
        self.is_initialized = True

