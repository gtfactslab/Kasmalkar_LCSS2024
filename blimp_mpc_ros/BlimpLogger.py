import csv
import numpy as np

class BlimpLogger:

    def __init__(self, filename):
        self.filename = filename

    def log(self, sim, ctrl):
        if ctrl.get_trajectory() is None:
            return
    
        with open('logs/' + self.filename, 'w', newline='') as outfile:
            writer = csv.writer(outfile)
       
            writer.writerow(['time',
                             'vx', 'vy', 'vz', 'wx', 'wy', 'wz', 'x', 'y', 'z', 'phi', 'theta', 'psi',
                             'vxdot', 'vydot', 'vzdot', 'wxdot', 'wydot', 'wzdot', 'xdot', 'ydot', 'zdot', 'phidot', 'thetadot', 'psidot',
                             'fx', 'fy', 'fz', 'tauz',
                             'x_ref', 'y_ref', 'z_ref', 'psi_ref',
                             'x_error', 'y_error', 'z_error', 'psi_error',
                             'solve_time',
                             'metadata'])
            
            # want timesteps x data elements

            n = sim.get_current_timestep()
            
            time_history = sim.get_time_vec().reshape((n,1))
            
            state_history = sim.get_state_history()
            
            state_dot_history = sim.get_state_dot_history()
            u_history = sim.get_full_u_history()
            
            trajectory_full = ctrl.get_trajectory()
            trajectory = trajectory_full[0:state_history.shape[0]]
            
            # the indices here are incorrect but no need to fix it
            error = trajectory - state_history[:, 0:4]
            solve_times = sim.get_solve_time_history().reshape((n, 1))
            
            ctrl_metadata = ctrl.get_metadata()
            metadata = np.concatenate((sim.get_current_timestep() * np.ones(1),
                                       sim.dT * np.ones(1),
                    np.pad(ctrl_metadata, ((0, n-len(ctrl_metadata)-2))))).reshape((n,1))

            data = np.hstack((time_history,
                              state_history,
                              state_dot_history,
                              u_history,
                              trajectory,
                              error,
                              solve_times,
                              metadata))
            
            for row in range(data.shape[0]):
                writer.writerow(np.asarray(data[row, :]).flatten())

            print("Wrote to " + str(self.filename))
