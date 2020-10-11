import numpy as np
import UR5_kinematics as UR_kin
from mujoco_py import functions
from numpy.linalg import inv
import our_func
import my_filter


def imic(sim, K, B, M, q_r, q_r_dot, p, p_dot, dt, f_in):
    start = p
    p_pos = p[0:3, 3]
	# Get current Jacobian
    j_muj, j_l_muj, _ = our_func.jacob(sim)

    x_r = UR_kin.forward(q_r)
    x_r = x_r[0:3, 3]
    x_r_dot = j_l_muj @ q_r_dot
    x_r_dot = x_r_dot[0:3]

    integrand = inv(M) * (-f_in + B @ (p_dot - x_r_dot) + K @ (p_pos - x_r))

    x_im_dot = x_r_dot + dt * integrand
    x_im_dot = np.diag(x_im_dot)
    x_im = x_r + dt * x_im_dot

    print('position error: ', p_pos - x_r)
    # print('inv(M): ', inv(M))
    print('x_im: ', x_im)
    print('x_im_dot: ', x_im_dot)
    print('Force: ', f_in)
    print('--------------------------------------------- \n')

    p[0:3, 3] = x_im
    try:
        q_d = UR_kin.inv_kin(start, p)
    except:
        print('Inverse Kinematics Failed!')
        q_d = q_r  

    try:
        q_d_dot = j_l_muj.T @ x_im_dot
    except:
        q_d_dot = q_r_dot
        print('Jacobian was singular! ', q_d[4])

    return q_d, q_d_dot
