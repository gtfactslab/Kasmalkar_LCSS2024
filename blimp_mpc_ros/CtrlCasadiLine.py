from . utilities import *
from . Trajectories import Trajectories
from . CtrlCasadi import CtrlCasadi

class CtrlCasadiLine(CtrlCasadi):
    def __init__(self, dT, skip_derivatives=True):
        super().__init__(dT)
        
    def init_sim(self, sim):
        x0 = sim.get_var('x')
        y0 = sim.get_var('y')
        z0 = sim.get_var('z')
        psi0 = sim.get_var('psi')
        
        trajectory = Trajectories.get_line(x0, y0, z0, psi0, self.dT)
        trajectory = Trajectories.get_generated(x0, y0, z0, psi0, self.dT)
        self.init_trajectory(trajectory)
        
        self.is_initialized = True

