from . BlimpController import BlimpController
import gurobipy as gp
from gurobipy import GRB
import numpy as np
import control
from . utilities import *
from . CtrlLQR import CtrlLQR
from . Trajectories import Trajectories
import math
import sys

class CtrlLQRTriangle(CtrlLQR):

    def __init__(self, dT, skip_derivatives=True):
        super().__init__(dT)

    def init_sim(self, sim):
        
        x0 = sim.get_var('x')
        y0 = sim.get_var('y')
        z0 = sim.get_var('z')
        psi0 = sim.get_var('psi')
        
        trajectory = Trajectories.get_triangle(x0, y0, z0, psi0, self.dT)
        self.init_trajectory(trajectory)
        
        self.is_initialized = True
