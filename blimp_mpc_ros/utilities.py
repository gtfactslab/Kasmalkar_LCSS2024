from scipy.spatial.transform import Rotation as R
import numpy as np
from math import pi

    
##  Operators
def H(r):
    return np.block([ [np.identity(3),         S(r).T],
                      [np.zeros((3, 3)),        np.identity(3)]])

def S(r):
    return np.block([[  0,       -r[2],    r[1]],
                     [r[2],     0,      -r[0]],
                     [-r[1],    r[0],      0]])

def C(M, nu):
    dimM = np.shape(M)[0]

    M11 = M[0:int(dimM/2), 0:int(dimM/2)]
    M12 = M[0:int(dimM/2), int(dimM/2):dimM]
    M21 = M[int(dimM/2):dimM, 0:int(dimM/2)]
    M22 = M[int(dimM/2):dimM, int(dimM/2):dimM]

    dimNu = np.shape(nu)[0]
    nu1 = nu[0:int(dimNu/2)]
    nu2 = nu[int(dimNu/2):dimNu]

    return np.block([[ np.zeros((3, 3)),    -S(M11@nu1 + M12@nu2)],
                     [-S(M11@nu1 + M12@nu2), -S(M21@nu1 + M22@nu2)]])


## Rotation matrices

def R_b__n(phi, theta, psi):

    x_rot = np.array([[1,         0,           0],
                      [0,       np.cos(phi),   -np.sin(phi)],
                      [0,       np.sin(phi),    np.cos(phi)]])

    y_rot = np.array([[np.cos(theta),      0,        np.sin(theta)],
                      [0,         1,           0],
                      [-np.sin(theta),     0,        np.cos(theta)]])
    
    z_rot = np.array([[np.cos(psi),    -np.sin(psi),       0],
                      [np.sin(psi),     np.cos(psi),       0],
                      [0,          0,           1]])

    # World-to-body
    return z_rot @ y_rot @ x_rot

def R_b__n_inv(phi, theta, psi):

    return np.array([[np.cos(psi)*np.cos(theta), np.cos(theta)*np.sin(psi), -np.sin(theta)],
                     [np.cos(psi)*np.sin(phi)*np.sin(theta) - np.cos(phi)*np.sin(psi), np.cos(phi)*np.cos(psi) + np.sin(phi)*np.sin(psi)*np.sin(theta), np.cos(theta)*np.sin(phi)],
                     [np.sin(phi)*np.sin(psi) + np.cos(phi)*np.cos(psi)*np.sin(theta), np.cos(phi)*np.sin(psi)*np.sin(theta) - np.cos(psi)*np.sin(phi), np.cos(phi)*np.cos(theta)]])

def T(phi, theta):
    
    return np.array([[1,     np.sin(phi)*np.tan(theta),      np.cos(phi)*np.tan(theta)],
                     [0,          np.cos(phi),                   -np.sin(phi)],
                     [0,     np.sin(phi)/np.cos(theta),      np.cos(phi)/np.cos(theta)]])

## Blimp model parameters

# Height of envelope Henv = 51 cm
Henv = 0.51

# Length of envelope Denv = 77 cm
Denv = 0.77

# Height of the gondola Hgon = 4 cm
Hgon = 0.04

# Distance from CB to motor thrust application point dvt
dvt = Henv/2 + Hgon

# Measured buoyant force = 0.725 N
FB_meas = 0.725

# Mass of envelope = 38.14 g
m_env = 0.03814

# Gravitational acceleration
g = 9.8

# Actual buoyant force
FB = FB_meas + m_env * g

# Density of air = 1.161 kg/m^3
rho_air = 1.161

# Volume of blimp envelope
Venv = FB / (rho_air * g)

# Mass of markers = 13.65 g
m_mkr = 0.01365

# Mass of gondola = 58.45 g
m_gon = 0.05845

# Mass of large battery = 14.65 g
m_largebat = 0.01465

# Mass of small battery = 8.29 g
m_smallbat = 0.00829

# Mass of helium
rho_he = 0.164
m_He = rho_he * Venv

# Mass of ballast = 1.81 g
m_blst = 0.00181

# Mass of blimp
# m1 = m_env + m_He + m_gon + m_mkr + m_blst + m_largebat
m = rho_air * Venv

# Venv2 = (m_gon + m_mkr + m_blst + m_env + m_largebat) / (rho_air - rho_he)

# Center of gravity to center of buoyancy
# r_z_g__b = 0.05 is excellent! was originally 0.09705 I think
# = 0.02 is also good (esp. for smaller batteries)
r_z_gb__b = 0.05
r_gb__b = np.array([0, 0, r_z_gb__b]).T
r_z_tg__b = dvt - r_z_gb__b

## Inertia matrix

m_Ax = 0.0466
m_Ay = m_Ax
m_Az = 0.0545
m_Axy = m_Ax

I_Ax = 0.0
I_Ay = I_Ax
I_Az = 0.0
I_Axy = I_Ax

M_A_CB = np.diag([m_Ax, m_Ay, m_Az, I_Ax, I_Ay, I_Az])

m_RB = 0.1249
I_RBx = 0.005821
I_RBy = I_RBx
I_RBz = I_RBx
I_RBxy = I_RBx

M_RB_CG = np.diag([m_RB, m_RB, m_RB, I_RBx, I_RBy, I_RBz])

M_RB_CB = H(r_gb__b).T @ M_RB_CG @ H(r_gb__b)

M_CB = M_RB_CB + M_A_CB

M_CB_inv = np.linalg.inv(M_CB)

m_x = m_RB + m_Ax
m_y = m_RB + m_Ay
m_z = m_RB + m_Az

I_x = I_RBx + m_RB * r_z_gb__b**2 + I_Ax
I_y = I_RBx + m_RB * r_z_gb__b**2 + I_Ay
I_z = I_RBz + I_Az

g_acc = 9.8
fg_n = m_RB * np.array([0, 0, g_acc]).T
f_g = fg_n[2]

## Aerodynamic damping
D_vx__CB = 0.0125
D_vy__CB = D_vx__CB
D_vz__CB = 0.0480
D_vxy__CB = D_vx__CB

D_wx__CB = 0.000862
D_wy__CB = D_wx__CB
D_wz__CB = D_wx__CB
D_omega_xy__CB = D_wx__CB
D_omega_z__CB = D_wz__CB

D_CB = np.diag([D_vx__CB, D_vy__CB, D_vz__CB, D_wx__CB, D_wy__CB, D_wz__CB])


## utility functions

def matrix_to_matlab(mat, name):

    print(name + ' = [', end='')
    for row in range(mat.shape[0]):
        if row > 0:
            print('\t', end='')
        for col in range(mat.shape[1]):
            print(mat[row][col], end=', ')
        
        if row < mat.shape[0] - 1:
            print(';')
    print('];')

def quat2euler(q):
    r = R.from_quat(q)
    return r.as_euler('XYZ', degrees=False)

def euler2quat(e):
    r = R.from_euler('xyz', e.T)
    return r.as_quat()
    
