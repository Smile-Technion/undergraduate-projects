import numpy as np
import UR5_kinematics as UR_kin
from mujoco_py import functions
from numpy.linalg import inv
import motion_plan
from scipy.spatial.transform import Rotation as R
import scipy.integrate as integrate
import angle_pack as ag


def derivs_imp(state, t, x_d, F, M, K, B):
    X_dot = np.zeros_like(state)

    X_dot[0:6] = state[6:]
    X_dot[6:] = inv(M) @ (K @ (x_d[0:6] - state[0:6]) + B @ (x_d[6:] - state[6:]) - F)

    return X_dot

def imic(sim, K, B, M, q_r, q_r_dot, p, p_dot, dt, f_in):
    # Full Instantaneous model - Force & Torque

    # Virtual position & orientation - X_0
    p_im = p.copy()
    start = p.copy()
    p_0 = ag.mat2pos(start)
    euler_dot_0 = np.zeros((1, 3))  # desired orientation is constant
    p_dot_0 = np.append(p_dot, euler_dot_0)

	# Get current Jacobian
    j_muj, j_l_muj, _ = motion_plan.jacob(sim)

    # Real position & orientation - X_r
    x_r_mat = UR_kin.forward(q_r)
    x_r = ag.mat2pos(x_r_mat)
    x_r_dot = j_muj @ q_r_dot
    fd = np.array([0, 5, 0, 0, 0, 0])
    integrand = inv(M) @ (-f_in - fd + B @ (p_dot_0 - x_r_dot) + K @ (p_0 - x_r))  # X_m_2dot

    # Discrete integral for X_m_dot & X_m
    x_im_dot = x_r_dot + dt * integrand
    x_im = x_r + dt * x_im_dot

    im_pos = x_im[0:3]
    im_euler = x_im[3:]
    p_im[0:3, 3] = im_pos
    rot_R_im = R.from_rotvec(im_euler)
    im_rot_mat = rot_R_im.as_matrix()
    p_im[0:3, 0:3] = im_rot_mat
    try:
        q_d = UR_kin.inv_kin(start, p_im)
    except:
        print('Inverse Kinematics Failed!')
        q_d = q_r  

    try:
        q_d_dot = j_muj.T @ x_im_dot
    except:
        q_d_dot = q_r_dot
        print('Jacobian was singular! ', q_d[4])

    x_im[0] = p_0[0]
    x_im[2] = p_0[2]

    return q_d, q_d_dot, x_im


def pbic(sim, K, B, M, state_temp, p, p_dot, t0, t1, f_in):
    tspan = [t0, t1]
    xd_mat = p.copy()
    start = p.copy()
    xd_pos = ag.mat2pos(xd_mat)
    euler_dot = np.zeros((1, 3))  # desired orientation is constant
    xd_dot = np.append(p_dot, euler_dot)
    xd = np.append(xd_pos, xd_dot)
    j_muj, j_l_muj, _ = motion_plan.jacob(sim)

    y = integrate.odeint(derivs_imp, state_temp, tspan, args=(xd, f_in, M, K, B))

    xm = y[1, :]
    im_pos = xm[0:3]
    im_euler = xm[3:6]
    x_im_dot = xm[6:]
    xd_mat[0:3, 3] = im_pos
    rot_R_im = R.from_rotvec(im_euler)
    im_rot_mat = rot_R_im.as_matrix()
    xd_mat[0:3, 0:3] = im_rot_mat
    try:
        q_d = UR_kin.inv_kin(start, xd_mat)
    except:
        print('Inverse Kinematics Failed!')
        q_d = UR_kin.inv_kin(start, start)

    try:
        q_d_dot = j_muj.T @ x_im_dot
    except:
        print('Jacobian was singular! ', q_d[4])
        q_d_dot = j_muj.T @ y[0, 6:]

    return q_d, q_d_dot, xm


