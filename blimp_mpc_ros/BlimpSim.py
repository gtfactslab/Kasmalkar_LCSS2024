import numpy as np
import scipy
import scipy.linalg
import time
import csv

from . utilities import *

class BlimpSim():
    def __init__(self, dT):
        self.dT = dT

        self.state_idx = {'vx': 0,
                    'vy': 1,
                    'vz': 2,
                    'wx': 3,
                    'wy': 4,
                    'wz': 5,
                    'x': 6,
                    'y': 7,
                    'z': 8,
                    'phi': 9,
                    'theta': 10,
                    'psi': 11}
        
        self.input_idx = {'fx': 0,
                          'fy': 1,
                          'fz': 2,
                          'tz': 3}

        self.current_timestep = 0


        self.state = np.zeros((12,1))
        self.state_dot = np.zeros((12,1))
        self.u = np.zeros((4,1))

        self.state_history = np.zeros((1,12))
        self.state_dot_history = np.zeros((1,12))
        self.u_history = np.zeros((1,4))
        self.time_vec = np.zeros(1)

        # The time it took to compute the control input
        # that led to the current state
        self.solve_time_history = np.zeros(1)

        self.start_time = 0
        self.time_delta = 0

        self.update_A_lin()
        self.update_A_dis()

        self.B_lin = np.array([
            [1/(m_Axy + m_RB), 0, 0, 0],
            [0, 1/(m_Axy + m_RB), 0, 0],
            [0, 0, 1/(m_Az + m_RB), 0],
            [0, -r_z_tg__b/(I_Axy + I_RBxy + m_RB*r_z_gb__b**2), 0, 0],
            [r_z_tg__b/(I_Axy + I_RBxy + m_RB*r_z_gb__b**2), 0, 0, 0],
            [0, 0, 0, 1/(I_Az + I_RBz)],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
        ])
       
        B_int = np.zeros((12,12))
        for i in range(10000):
            dTau = dT / 10000
            tau = i * dTau
            B_int += scipy.linalg.expm(self.A_lin * tau) * dTau
        self.B_dis = B_int @ self.B_lin #np.linalg.inv(A) @ (A_dis - np.eye(12)) @ B
    
    def update_model(self, u):
        self.u = u

        tau_B = np.array([u[0],
                            u[1],
                            u[2],
                            -r_z_tg__b * u[1],
                            r_z_tg__b * u[0],
                            u[3]]).reshape((6,1))
        
        x = self.get_var('x')
        y = self.get_var('y')
        z = self.get_var('z')
        phi = self.get_var('phi')
        theta = self.get_var('theta')
        psi = self.get_var('psi')

        eta_bn_n = np.array([x, y, z, phi, theta, psi]).reshape((6,1))
        
        vx = self.get_var('vx')
        vy = self.get_var('vy')
        vz = self.get_var('vz')
        wx = self.get_var('wx')
        wy = self.get_var('wy')
        wz = self.get_var('wz')

        nu_bn_b = np.array([vx, vy, vz, wx, wy, wz]).reshape((6,1))

        # Restoration torque
        fg_B = R_b__n_inv(phi, theta, psi) @ fg_n
        g_CB = -np.block([[np.zeros((3, 1))],
                        [np.reshape(np.cross(r_gb__b, fg_B), (3, 1))]])

        # Update state
        eta_bn_n_dot = np.block([[R_b__n(phi, theta, psi),    np.zeros((3, 3))],
                                [np.zeros((3, 3)),            T(phi, theta)]]) @ nu_bn_b
        
        nu_bn_b_dot = np.reshape(-M_CB_inv @ (C(M_CB, nu_bn_b) @ nu_bn_b + \
                            D_CB @ nu_bn_b + g_CB - tau_B), (6, 1))
        
        eta_bn_n = eta_bn_n + eta_bn_n_dot * self.dT
        nu_bn_b = nu_bn_b + nu_bn_b_dot * self.dT

        self.state_dot = np.vstack((nu_bn_b_dot, eta_bn_n_dot))
        self.state = np.vstack((nu_bn_b, eta_bn_n))

        self.update_history()

    def update_A_lin(self):
        # required because any psi is an equilibrium psi
        psi = self.get_var('psi')
        
        self.A_lin = np.array([
            [-D_vxy__CB/(m_Axy + m_RB), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, -D_vxy__CB/(m_Axy + m_RB), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, -D_vz__CB/(m_Az + m_RB), 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, -D_omega_xy__CB/(I_Axy + I_RBxy + m_RB*r_z_gb__b**2), 0, 0, 0, 0, 0, -(f_g*r_z_gb__b)/(I_Axy + I_RBxy + m_RB*r_z_gb__b**2), 0, 0],
            [0, 0, 0, 0, -D_omega_xy__CB/(I_Axy + I_RBxy + m_RB*r_z_gb__b**2), 0, 0, 0, 0, 0, -(f_g*r_z_gb__b)/(I_Axy + I_RBxy + m_RB*r_z_gb__b**2), 0],
            [0, 0, 0, 0, 0, -D_omega_z__CB/(I_Az + I_RBz), 0, 0, 0, 0, 0, 0],
            [np.cos(psi), -np.sin(psi), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [np.sin(psi), np.cos(psi), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
        ])

    def update_A_dis(self):
        self.update_A_lin()
        self.A_dis = scipy.linalg.expm(self.A_lin * self.dT)
    
    def get_A_lin(self):
        self.update_A_lin()
        return self.A_lin
    
    def get_A_dis(self):
        self.update_A_dis()
        return self.A_dis
    
    def get_B_lin(self):
        return self.B_lin
    
    def get_B_dis(self):
        return self.B_dis
    
    def start_timer(self):
        self.start_time = time.process_time_ns()

    def end_timer(self):
        self.time_delta = time.process_time_ns() - self.start_time
        
    def set_var(self, var, val):
        self.state[self.state_idx[var]] = val

    def set_var_dot(self, var, val):
        self.state_dot[self.state_idx[var]] = val

    def get_var(self, var):
        return self.state[self.state_idx[var]].item()
    
    def get_var_dot(self, var):
        return self.state_dot[self.state_idx[var]].item()
    
    def get_var_history(self, var):
        return self.state_history[:, self.state_idx[var]]
    
    def get_var_dot_history(self, var):
        return self.state_dot_history[:, self.state_idx[var]]
    
    def get_full_u_history(self):
        return self.u_history

    def get_u_history(self, var):
        return self.u_history[:, self.input_idx[var]]
    
    def get_state(self):
        return self.state
    
    def get_state_history(self):
        return self.state_history
    
    def get_state_dot_history(self):
        return self.state_dot_history
    
    def get_state_dot(self):
        return self.state_dot
    
    def get_body_x(self, length):
        return R_b__n(self.get_var('phi'),
                      self.get_var('theta'),
                      self.get_var('psi')) \
                        @ np.array([length, 0, 0]).T
    
    def get_body_y(self, length):
        return R_b__n(self.get_var('phi'),
                      self.get_var('theta'),
                      self.get_var('psi')) \
                        @ np.array([0, length, 0]).T
    
    def get_body_z(self, length):
        return R_b__n(self.get_var('phi'),
                      self.get_var('theta'),
                      self.get_var('psi')) \
                        @ np.array([0, 0, length]).T
    
    def get_time_vec(self):
        return self.time_vec
    
    def get_solve_time_history(self):
        return self.solve_time_history
    
    def get_current_timestep(self):
        return self.current_timestep
    
    def get_current_time(self):
        return self.current_timestep * self.dT

    def update_history(self):
        self.current_timestep += 1

        if self.current_timestep == 1:
            self.solve_time_history = np.array([self.time_delta])
            self.state_history = self.state.reshape((1,12))
            self.state_dot_history = self.state_dot.reshape((1,12))
            self.u_history = self.u.reshape((1,4))
            self.time_vec = np.array([self.current_timestep * self.dT])
        else:
            self.solve_time_history = np.append(self.solve_time_history, self.time_delta)
            self.state_history = np.append(self.state_history, self.state.reshape((1,12)), axis=0)
            self.state_dot_history = np.append(self.state_dot_history,
                                 self.state_dot.reshape((1,12)), axis=0)
            self.u_history = np.append(self.u_history, self.u.reshape((1,4)), axis=0)
            self.time_vec = np.append(self.time_vec, self.current_timestep * self.dT)

    def load_data(self, filename):
        with open('logs/' + filename, 'r') as infile:
            reader = csv.reader(infile)
            csv_data_np = np.array(list(reader))

            self.current_timestep = int(float(csv_data_np[1, 38].item()))
            self.dT = float(csv_data_np[2, 38])
            
            data_np = csv_data_np[1:, 0:38]
            
            data_float = np.array([[float(i) for i in j] for j in data_np])

            self.time_vec = data_float[:, 0]

            self.state_history = data_float[:, 1:13]
            self.state = self.state_history[self.current_timestep-1, :]
   
            self.state_dot_history = data_float[:, 13:25]
            self.state_dot = self.state_dot_history[self.current_timestep-1, :]

            self.u_history = data_float[:, 25:29]
            self.u = self.u_history[self.current_timestep-1, :]

            self.trajectory = data_float[:, 29:33]
            
            self.error_history = data_float[:, 33:37]

            self.solve_time_history = data_float[:, 37]
            
            
