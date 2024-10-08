import numpy as np
import time
from . BlimpController import BlimpController
from . utilities import *
from . BlimpSim import BlimpSim
import control
import casadi

class CtrlCasadi(BlimpController):
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

        # Re-define in an 8x8 matrix without pitch and roll
        self.Q = np.array([
            [1/(max_vx_err**2), 0, 0, 0, 0, 0, 0, 0],       # vx
            [0, 1/(max_vy_err**2), 0, 0, 0, 0, 0, 0],       # vy
            [0, 0, 1/(max_vz_err**2), 0, 0, 0, 0, 0],       # vz
            [0, 0, 0, 1/(max_wz_err**2), 0, 0, 0, 0],       # wz
            [0, 0, 0, 0, 1/(max_x_err**2), 0, 0, 0],       # x
            [0, 0, 0, 0, 0, 1/(max_y_err**2), 0, 0],       # y
            [0, 0, 0, 0, 0, 0, 1/(max_z_err**2), 0],       # z
            [0, 0, 0, 0, 0, 0, 0, 10/(max_psi_err**2)],       # psi
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

        # Setup controller.
        N = 10 # MPC horizon length

        self.opti = casadi.Opti()
        self.opt_u = self.opti.variable(4, N)
        self.opt_x = self.opti.variable(12, N+1)

        # u_ref = self.opti.parameter(N, 4)
        self.y_ref = self.opti.parameter(8, N) # In the output space.
        self.opt_x_t = self.opti.parameter(12,) # Initialization

        opts_setting = {
            'ipopt.max_iter': 2000
        }
        
        self.opti.solver('ipopt', opts_setting)

        # Initial condition.
        self.opti.subject_to( self.opt_x[:,0] == self.opt_x_t )

        # Create the dynamics equality constraints
        for j in range(N):
            x_next = self.opt_x[:, j] + dT * BlimpSim.f(self.opt_x[:, j], self.opt_u[:, j])
            self.opti.subject_to( self.opt_x[:, j+1] == x_next )

        # Create the cost function.
        obj = 0
        gamma = .1 # Weighting on the control effort.
        for i in range(N):
            state_error = self.opt_x[[0,1,2,5,6,7,8,11], i+1] - self.y_ref[:, i]
            obj += casadi.mtimes([state_error.T, self.Q, state_error])
            # Penalize the control effort.
            obj += gamma * casadi.mtimes([self.opt_u[:, i].T, self.R, self.opt_u[:, i]])
        
        self.opti.minimize(obj)

        # Bound the control input variables.
        self.opti.subject_to( self.opti.bounded(-max_fx, self.opt_u[0, :], max_fx) )
        self.opti.subject_to( self.opti.bounded(-max_fy, self.opt_u[1, :], max_fy) )
        self.opti.subject_to( self.opti.bounded(-max_fz, self.opt_u[2, :], max_fz) )
        self.opti.subject_to( self.opti.bounded(-max_tz, self.opt_u[3, :], max_tz) )
        
        self.metadata = np.array([
            str(self.Q),
            str(self.R),
            str(N)
        ])

        self.N = N
        
    def get_ctrl_action(self, sim):
        sim.start_timer()
        
        n = sim.get_current_timestep()
        
        if n > len(self.traj_x):
            return None
        
        # Solve the NLMPC problem here.
        # Update parameter values with references.

        # Stack the reference trajectory.
        traj_ref = np.vstack((
            self.traj_x_dot[n:n+self.N],
            self.traj_y_dot[n:n+self.N],
            self.traj_z_dot[n:n+self.N],
            self.traj_psi_dot[n:n+self.N],
            self.traj_x[n:n+self.N],
            self.traj_y[n:n+self.N],
            self.traj_z[n:n+self.N],
            self.traj_psi[n:n+self.N],
        ))

        print('[INFO] made it to set_value for the reference trajectory')
        self.opti.set_value(self.y_ref, traj_ref)

        # Stack sim.get_var('x') to sim.get_var('psi') to get the current state.
        x_t = np.array([
            sim.get_var('vx'),
            sim.get_var('vy'),
            sim.get_var('vz'),
            sim.get_var('wx'),
            sim.get_var('wy'),
            sim.get_var('wz'),
            sim.get_var('x'),
            sim.get_var('y'),
            sim.get_var('z'),
            sim.get_var('phi'),
            sim.get_var('theta'),
            sim.get_var('psi')
        ])

        # set_initial provides an initial guess, NOT an initial condition!
        # self.opti.set_initial(self.opt_x, x_t)
        self.opti.set_value(self.opt_x_t, x_t.reshape((1,12)))

        solution = self.opti.solve()

        u = solution.value(self.opt_u[:, 0])
        print(u)
        x_predicted = solution.value(self.opt_x)

        sim.end_timer()

        return u.reshape((4,1))
