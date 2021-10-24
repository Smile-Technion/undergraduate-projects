import numpy as np
import cmath

# PB-IC example
zeta_p, omega_p, G_p = 4.3205, 21.6025, 1/7.5
zeta_m, omega_m, G_m = 0.5401, 10.8012, 1/30
dt = 0.01
t_s = 10*dt
zeta_PD = 0.7
omega_PD = 4.6/(zeta_PD*t_s)

P_PB = omega_PD**2
D_PB = 2*zeta_PD*np.sqrt(P_PB)
omega_PB = np.sqrt(P_PB)
zeta_PB = (1/2)*D_PB/np.sqrt(P_PB)

omega_DB = omega_m
zeta_DB = zeta_m

lambda_PB = [complex(-omega_PB*zeta_PB, omega_PB*np.sqrt(1-zeta_PB**2)), complex(-omega_PB*zeta_PB, -omega_PB*np.sqrt(1-zeta_PB**2))]
lambda_DB = [complex(-omega_DB*zeta_DB, omega_DB*np.sqrt(1-zeta_DB**2)), complex(-omega_DB*zeta_DB, -omega_DB*np.sqrt(1-zeta_DB**2))]

print(lambda_PB)
