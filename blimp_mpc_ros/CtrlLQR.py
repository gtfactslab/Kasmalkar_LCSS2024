import gurobipy as gp
from gurobipy import GRB
import numpy as np
import time
from . BlimpController import BlimpController
from . Trajectories import Trajectories
from . utilities import *
import control
import sys

class CtrlLQR(BlimpController):

    def __init__(self, dT):
        super().__init__(dT)
        
        max_vx_err = 0.1
        max_vy_err = 0.1
        max_vz_err = 0.1
        
        max_wx_err = 10
        max_wy_err = 10
        max_wz_err = 5
        
        max_x_err = 0.05
        max_y_err = 0.05
        max_z_err = 0.05
        
        max_phi_err = 10 * np.pi/180
        max_th_err = 10 * np.pi/180
        max_psi_err = 10 * np.pi/180
        
        self.Q = np.array([
        	[1/(max_vx_err**2), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],       # vx
        	[0, 1/(max_vy_err**2), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],       # vy
        	[0, 0, 1/(max_vz_err**2), 0, 0, 0, 0, 0, 0, 0, 0, 0],       # vz
        	[0, 0, 0, 1/(max_wx_err**2), 0, 0, 0, 0, 0, 0, 0, 0],       # wx
        	[0, 0, 0, 0, 1/(max_wy_err**2), 0, 0, 0, 0, 0, 0, 0],       # wy
        	[0, 0, 0, 0, 0, 1/(max_wz_err**2), 0, 0, 0, 0, 0, 0],       # wz
        	[0, 0, 0, 0, 0, 0, 1/(max_x_err**2), 0, 0, 0, 0, 0],       # x
        	[0, 0, 0, 0, 0, 0, 0, 1/(max_y_err**2), 0, 0, 0, 0],       # y
        	[0, 0, 0, 0, 0, 0, 0, 0, 1/(max_z_err**2), 0, 0, 0],       # z
        	[0, 0, 0, 0, 0, 0, 0, 0, 0, 1/(max_phi_err**2), 0, 0],       # phi
        	[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1/(max_th_err**2), 0],       # theta
        	[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1/(max_psi_err**2)],       # psi
        ])
        
        max_fx = 0.035
        max_fy = 0.035
        max_fz = 0.2
        max_tz = 0.001
        
        self.R = np.array([
        	[1/(max_fx**2), 0, 0, 0],
        	[0, 1/(max_fy**2), 0, 0],
        	[0, 0, 1/(max_fz**2), 0],
        	[0, 0, 0, 1/(max_tz**2)]
        ]) * 100
        
        
        self.metadata = np.array([
            str(self.Q),
            str(self.R)
        ])
        
    def get_ctrl_action(self, sim):
        
        self.K = control.lqr(sim.get_A_lin(), sim.get_B_lin(), self.Q, self.R)[0]
        
        sim.start_timer()
        
        n = sim.get_current_timestep()
        
        if n > len(self.traj_x):
            return None

        error = np.array([
            sim.get_var('vx') - self.traj_x_dot[n],
            sim.get_var('vy') - self.traj_y_dot[n],
            sim.get_var('vz') - self.traj_z_dot[n],
            sim.get_var('wx'),
            sim.get_var('wy'),
            sim.get_var('wz') - self.traj_psi_dot[n],
            sim.get_var('x') - self.traj_x[n],
            sim.get_var('y') - self.traj_y[n],
            sim.get_var('z') - self.traj_z[n],
            sim.get_var('phi'),
            sim.get_var('theta'),
            sim.get_var('psi') - self.traj_psi[n]
        ])
        
        u = (-self.K @ error.reshape((12,1))).reshape((4,1))

        sim.end_timer()

        return u.reshape((4,1))
