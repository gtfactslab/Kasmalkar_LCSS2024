import numpy as np
import matplotlib.pyplot as plt
import csv

class Trajectories:
    
    def get_helix(x0, y0, z0, psi0, dT):
        # Time
        TRACKING_TIME = 40
        SETTLE_TIME = 20

        tracking_time = np.arange(0, TRACKING_TIME, dT)
        settle_time = np.arange(TRACKING_TIME, TRACKING_TIME + SETTLE_TIME + 1, dT)

        time_vec = np.concatenate((tracking_time, settle_time))

        At = 0.75
        z_max = -0.5

        # Trajectory definition
        f = 1/TRACKING_TIME
        
        z_slope = z_max / TRACKING_TIME
        
        traj_x = np.concatenate((x0 + (At - At * np.cos(2*np.pi*f*tracking_time)), x0 * np.ones(len(settle_time))))
        traj_y = np.concatenate((y0 + At * np.sin(2*np.pi*f*tracking_time), y0 * np.ones(len(settle_time))))
        traj_z = np.concatenate((z0 + tracking_time * z_slope, (z0 + TRACKING_TIME * z_slope) * np.ones(len(settle_time))))
        traj_psi = np.concatenate((psi0 * np.ones(len(tracking_time)), psi0 * np.ones(len(settle_time))))
        
        traj_x_dot = np.concatenate((2*np.pi*f*At*np.sin(2*np.pi*f*tracking_time), np.zeros(len(settle_time))))
        traj_y_dot = np.concatenate((2*np.pi*f*At*np.cos(2*np.pi*f*tracking_time), np.zeros(len(settle_time))))
        traj_z_dot = np.concatenate((z_slope * np.ones(len(tracking_time)), np.zeros(len(settle_time))))
        traj_psi_dot = np.concatenate((np.zeros(len(tracking_time)), np.zeros(len(settle_time))))

        traj_x_ddot = np.concatenate((-(2*np.pi*f)**2*At*np.cos(2*np.pi*f*tracking_time), np.zeros(len(settle_time))))
        traj_y_ddot = np.concatenate((-(2*np.pi*f)**2*At*np.sin(2*np.pi*f*tracking_time), np.zeros(len(settle_time))))
        traj_z_ddot = np.concatenate((np.zeros(len(tracking_time)), np.zeros(len(settle_time))))
        traj_psi_ddot = np.concatenate((np.zeros(len(tracking_time)), np.zeros(len(settle_time))))
        
        return (traj_x, traj_y, traj_z, traj_psi, traj_x_dot, traj_y_dot, traj_z_dot, traj_psi_dot, traj_x_ddot, traj_y_ddot, traj_z_ddot, traj_psi_ddot)
        
        
    def get_line(x0, y0, z0, psi0, dT):
        # Trajectory definition
        TRACKING_TIME = 40
        SETTLE_TIME = 20
        
        m = 0.05

        tracking_time = np.arange(0, TRACKING_TIME, dT)
        settle_time = np.arange(TRACKING_TIME, TRACKING_TIME + SETTLE_TIME + 1, dT)

        time_vec = np.concatenate((tracking_time, settle_time))
        
        traj_x = np.concatenate((x0 + m*tracking_time, (x0 + m*TRACKING_TIME) * np.ones(len(settle_time))))
        traj_y = np.concatenate((y0 * np.ones(len(tracking_time)), y0 * np.ones(len(settle_time))))
        traj_z = np.concatenate((z0 * np.ones(len(tracking_time)), z0 * np.ones(len(settle_time))))
        traj_psi = np.concatenate((psi0 * np.ones(len(tracking_time)), psi0 * np.ones(len(settle_time))))
        
        traj_x_dot = np.concatenate((m * np.ones(len(tracking_time)), np.zeros(len(settle_time))))
        traj_y_dot = np.concatenate((np.zeros(len(tracking_time)), np.zeros(len(settle_time))))
        traj_z_dot = np.concatenate((np.zeros(len(tracking_time)), np.zeros(len(settle_time))))
        traj_psi_dot = np.concatenate((np.zeros(len(tracking_time)), np.zeros(len(settle_time))))
        
        traj_x_ddot = np.concatenate((np.zeros(len(tracking_time)), np.zeros(len(settle_time))))
        traj_y_ddot = np.concatenate((np.zeros(len(tracking_time)), np.zeros(len(settle_time))))
        traj_z_ddot = np.concatenate((np.zeros(len(tracking_time)), np.zeros(len(settle_time))))
        traj_psi_ddot = np.concatenate((np.zeros(len(tracking_time)), np.zeros(len(settle_time))))
        
        return (traj_x, traj_y, traj_z, traj_psi, traj_x_dot, traj_y_dot, traj_z_dot, traj_psi_dot, traj_x_ddot, traj_y_ddot, traj_z_ddot, traj_psi_ddot)
        
        
    def get_triangle(x0, y0, z0, psi0, dT):
        # Trajectory definition
        TRACKING_TIME = 180

        x_distance = 0.5
        y_distance = 0.5
        z_distance = -0.5

        time_vec = np.arange(0, TRACKING_TIME, dT)

        traj_x = np.empty(len(time_vec))
        traj_y = np.empty(len(time_vec))
        traj_z = np.empty(len(time_vec))
        
        traj_x_dot = np.empty(len(time_vec))
        traj_y_dot = np.empty(len(time_vec))
        traj_z_dot = np.empty(len(time_vec))
        
        traj_psi = psi0 * np.ones(len(time_vec))
        traj_psi_dot = np.zeros(len(time_vec))
        
        for i in range(0, int(len(time_vec)/4)):
            traj_x[i] = x0 + (x_distance) / (TRACKING_TIME/4) * time_vec[i]
            traj_y[i] = y0
            
            traj_x_dot[i] = (x_distance) / (TRACKING_TIME/4)
            traj_y_dot[i] = 0
            
            if i < len(time_vec)/8:
                traj_z[i] = z0 + (z_distance) / (TRACKING_TIME/8) * time_vec[i]
                traj_z_dot[i] = (z_distance) / (TRACKING_TIME/8)
            else:
                traj_z[i] = z0 + z_distance - (z_distance) / (TRACKING_TIME/8) * (time_vec[i] - time_vec[int(len(time_vec)/8)])
                traj_z_dot[i] = -(z_distance) / (TRACKING_TIME/8)
            
            
        for i in range(int(len(time_vec)/4) , int(len(time_vec)/2)):
            traj_x[i] = x0 + x_distance
            traj_y[i] = y0 + (y_distance) / (TRACKING_TIME/4) * (time_vec[i] - time_vec[int(len(time_vec)/4)])
            
            traj_x_dot[i] = 0
            traj_y_dot[i] = (y_distance) / (TRACKING_TIME/4)
            
            if i < len(time_vec)*3/8:
                traj_z[i] = z0 + (z_distance) / (TRACKING_TIME/8) * (time_vec[i] - time_vec[int(len(time_vec)/4)])
                traj_z_dot[i] = (z_distance) / (TRACKING_TIME/8)
            else:
                traj_z[i] = z0 + z_distance - (z_distance) / (TRACKING_TIME/8) * (time_vec[i] - time_vec[int(len(time_vec)*3/8)])
                traj_z_dot[i] = -(z_distance) / (TRACKING_TIME/8)
            
        for i in range(int(len(time_vec)/2), int(len(time_vec)*3/4)):
            traj_x[i] = x0 + x_distance - (x_distance) / (TRACKING_TIME/4) * (time_vec[i] - time_vec[int(len(time_vec)/2)])
            traj_y[i] = y0 + y_distance
            
            traj_x_dot[i] = - (x_distance) / (TRACKING_TIME/4)
            traj_y_dot[i] = 0
            
            if i < len(time_vec)*5/8:
                traj_z[i] = z0 + (z_distance) / (TRACKING_TIME/8) * (time_vec[i] - time_vec[int(len(time_vec)/2)])
                traj_z_dot[i] = (z_distance) / (TRACKING_TIME/8)
            else:
                traj_z[i] = z0 + z_distance - (z_distance) / (TRACKING_TIME/8) * (time_vec[i] - time_vec[int(len(time_vec)*5/8)])
                traj_z_dot[i] = -(z_distance) / (TRACKING_TIME/8)
                
        for i in range(int(len(time_vec)*3/4), int(len(time_vec))):
            traj_x[i] = x0
            traj_y[i] = y0 + y_distance - (y_distance) / (TRACKING_TIME/4) * (time_vec[i] - time_vec[int(len(time_vec)*3/4)])
            
            traj_x_dot[i] = 0
            traj_y_dot[i] = -(y_distance) / (TRACKING_TIME/4)
            
            if i < len(time_vec)*7/8:
                traj_z[i] = z0 + (z_distance) / (TRACKING_TIME/8) * (time_vec[i] - time_vec[int(len(time_vec)*3/4)])
                traj_z_dot[i] = (z_distance) / (TRACKING_TIME/8)
            else:
                traj_z[i] = z0 + z_distance - (z_distance) / (TRACKING_TIME/8) * (time_vec[i] - time_vec[int(len(time_vec)*7/8)])
                traj_z_dot[i] = -(z_distance) / (TRACKING_TIME/8)
        
        traj_x_ddot = np.zeros(len(time_vec))
        traj_y_ddot = np.zeros(len(time_vec))
        traj_z_ddot = np.zeros(len(time_vec))
        traj_psi_ddot = np.zeros(len(time_vec))
        
        return (traj_x, traj_y, traj_z, traj_psi, traj_x_dot, traj_y_dot, traj_z_dot, traj_psi_dot, traj_x_ddot, traj_y_ddot, traj_z_ddot, traj_psi_ddot)
        
