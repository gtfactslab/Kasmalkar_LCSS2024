import gurobipy as gp
from gurobipy import GRB
import numpy as np
import time
from . CtrlFBL import CtrlFBL
from . Trajectories import Trajectories
from . utilities import *
import sys

class CtrlFBLHelix(CtrlFBL):

    def __init__(self, dT):
        super().__init__(dT)
        
        self.metadata = np.array([
            f"k1 = {self.k1.T}",
            f"k2 = {self.k2.T}",
            f"dT = {dT}"
        ])
        
    def init_sim(self, sim):
        
        x0 = sim.get_var('x')
        y0 = sim.get_var('y')
        z0 = sim.get_var('z')
        psi0 = sim.get_var('psi')
        
        trajectory = Trajectories.get_helix(x0, y0, z0, psi0, self.dT)
        self.init_trajectory(trajectory)
        
        self.is_initialized = True
