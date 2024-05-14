import numpy as np
import csv

class BlimpController():

    def __init__(self, dT, skip_derivatives=False):
        self.dT = dT
        self.error_history = None
        self.is_initialized = False
        
        self.traj_x = None
        self.traj_y = None
        self.traj_z = None
        self.traj_psi = None
        
        self.metadata = None

        self.order = 12
        self.num_inputs = 4
        self.num_outputs = 6
        
    def init_trajectory(self, trajectory):
        self.traj_x         = trajectory[0]
        self.traj_y         = trajectory[1]
        self.traj_z         = trajectory[2]
        self.traj_psi       = trajectory[3]
        
        self.traj_x_dot     = trajectory[4]
        self.traj_y_dot     = trajectory[5]
        self.traj_z_dot     = trajectory[6]
        self.traj_psi_dot   = trajectory[7]
        
        self.traj_x_ddot    = trajectory[8]
        self.traj_y_ddot    = trajectory[9]
        self.traj_z_ddot    = trajectory[10]
        self.traj_psi_ddot  = trajectory[11]
        
        
    def get_ctrl_action(self, sim):
        pass

    def init_sim(self, sim):
        pass

    def get_trajectory(self):
        return np.array([
                self.traj_x,
                self.traj_y,
                self.traj_z,
                self.traj_psi
        ]).T
    
    def get_metadata(self):
        return self.metadata
