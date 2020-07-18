import UR5_kinematics as UR_kin
import numpy as np
from matplotlib import pyplot as plt
import motion_plan_saar as mp
from mujoco_py import functions
from numpy.linalg import inv
import dimanic_mat as dm
import my_filter
import mujoco_py


def get_joints(sim, q_r, q_r_dot):
    joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
              'wrist_3_joint']

    for joint in range(len(joints)):
        q_r[joint] = sim.data.get_joint_qpos(joints[joint])
        q_r_dot[joint] = sim.data.get_joint_qvel(joints[joint])

    return q_r, q_r_dot


def execute_time(start_point, end_point, speed):
    # calculate execute time based on distance between the points
    distance = np.linalg.norm(end_point - start_point, 2)
    ex_time = round(distance / speed, 2)
    if ex_time < 1:
        return ex_time * 3
    else:
        return ex_time


def xyz_velocity(start_point, end_point, time_to_execut, time_diff):
    # convenient, don't really need this
    T = time_to_execut

    # create time vector
    reso = int(T / time_diff)
    t = np.linspace(0, T, reso)

    # --- linear movement of end effector --- #
    # coefficients
    a2 = 30
    a3 = -60 / T
    a4 = 30 / (T ** 2)
    t2 = np.power(t, 2)
    t3 = np.power(t, 3)
    t4 = np.power(t, 4)
    t_coef = a4 * t4 + a3 * t3 + a2 * t2

    # path output init
    p_dot = np.zeros((3, len(t)))

    # get position vectors
    v0 = start_point[0:3, -1]
    vf = end_point[0:3, -1]

    for k in range(3):
        coeff = (vf[k] - v0[k]) / (T ** 3)
        p_dot[k, :] = coeff * t_coef

    return p_dot


def xyz_acc(start_point, end_point, time_to_execut, time_diff):
    # convenient, don't really need this
    T = time_to_execut

    # create time vector
    reso = int(T / time_diff)
    t = np.linspace(0, T, reso)

    # --- linear movement of end effector --- #
    # coefficients
    a1 = 60
    a2 = -180 / T
    a3 = 120 / (T ** 2)
    t2 = np.power(t, 2)
    t3 = np.power(t, 3)
    t_coef = a3 * t3 + a2 * t2 + a1 * t

    # path output init
    p_2dot = np.zeros((3, len(t)))

    # get position vectors
    v0 = start_point[0:3, -1]
    vf = end_point[0:3, -1]

    for k in range(3):
        coeff = (vf[k] - v0[k]) / (T ** 3)
        p_2dot[k, :] = coeff * t_coef

    return p_2dot


# ---- build path in Cartesian space ---- #
def build_cartesian_path(path, move_speed, time_diff):
    # First leg of path
    ex_time = execute_time(path[0], path[1], move_speed)
    xyz_path = mp.motion_plan(path[0], path[1], ex_time, time_diff)
    p_dot = xyz_velocity(path[0], path[1], ex_time, time_diff)
    p_2dot = xyz_acc(path[0], path[1], ex_time, time_diff)

    # Rest of the path
    for k in range(1, len(path) - 1):
        ex_time = execute_time(path[k], path[k + 1], move_speed)
        next_part = mp.motion_plan(path[k], path[k + 1], ex_time, time_diff)
        xyz_path = np.concatenate((xyz_path, next_part), axis=0)

        # velocity
        next_p_dot = xyz_velocity(path[k], path[k + 1], ex_time, time_diff)
        p_dot = np.concatenate((p_dot, next_p_dot), axis=1)

        # acceleration
        next_p_2dot = xyz_acc(path[k], path[k + 1], ex_time, time_diff)
        p_2dot = np.concatenate((p_2dot, next_p_2dot), axis=1)

    return xyz_path, p_dot, p_2dot


# --------------------------------------- #


def build_trajectory(xyz_path):
    q_path = np.zeros((6, len(xyz_path[:, 3, 3])))
    for k in range(len(xyz_path[:, 3, 3])):
        q_path[:, k] = UR_kin.inv_kin(xyz_path[k - 1], xyz_path[k])
    return q_path


def build_target(x_desired, r_desired):
    # x_desired = np.array([x, y, z])
    #
    # r_desired = np.array([[1, 0,  0],
    #                       [0, 0,  1],
    #                       [0, -1, 0]])

    v_zero = np.array([[0, 0, 0]])
    r_desired = np.concatenate((r_desired, v_zero))
    x_desired = np.append(x_desired, 1)
    x_desired = np.reshape(x_desired, (4, 1))
    target = np.concatenate((r_desired, x_desired), axis=1)
    return target


def jacob(sim):
    target_jacp = np.zeros(3 * sim.model.nv)
    target_jacr = np.zeros(3 * sim.model.nv)
    sim.data.get_site_jacp('gripperpalm', jacp=target_jacp)
    sim.data.get_site_jacr('gripperpalm', jacr=target_jacr)
    J_L = target_jacp.reshape((3, sim.model.nv))
    J_A = target_jacr.reshape((3, sim.model.nv))

    J_L = J_L[:, 6:12]
    J_A = J_A[:, 6:12]
    J = np.concatenate((J_L, J_A), axis=0)
    return J, J_L, J_A


def q_velocity(sim, p_dot, q_path):

    for i in range(len(p_dot[0, :])):
        j_muj, j_l_muj, _ = jacob(sim)  # for test

        j = dm.Jacobian(q_path[:, i])
        j_l = j[0:3, :]
        q_dot_next = j_l.T @ p_dot[:, i]
        q_dot_next = np.reshape(q_dot_next, (6, 1))
        if i:
            q_dot = np.concatenate((q_dot, q_dot_next), axis=1)
        else:
            q_dot = q_dot_next

    return q_dot


def H(sim):
    J, J_L, J_A = jacob(sim)
    H = np.zeros(sim.model.nv * sim.model.nv)
    functions.mj_fullM(sim.model, H, sim.data.qM)
    H_L = np.dot(np.linalg.pinv(J_L.T), np.dot(H.reshape(sim.model.nv, sim.model.nv), np.linalg.pinv(J_L)))
    H_all = np.dot(np.linalg.pinv(J.T), np.dot(H.reshape(sim.model.nv, sim.model.nv), np.linalg.pinv(J)))
    return H_all, J


def print_q(q1, q2, dt, headline):
    q = np.concatenate((q1, q2), axis=1)
    t_len = len(q[0, :]) * dt
    t = np.linspace(0, t_len, len(q[0, :]))

    f, (ax1, ax2, ax3, ax4, ax5, ax6) = plt.subplots(6, 1, sharex='col')
    if headline == 'angle':
        ax1.set_title('desired joints angle[rad]')
    elif headline == 'speed':
        ax1.set_title('desired joints speed[rad/sec]')

    ax1.plot(t, q[0, :])
    ax1.set_ylabel('q1')
    ax2.plot(t, q[1, :])
    ax2.set_ylabel('q2')
    ax3.plot(t, q[2, :])
    ax3.set_ylabel('q3')
    ax4.plot(t, q[3, :])
    ax4.set_ylabel('q4')
    ax5.plot(t, q[4, :])
    ax5.set_ylabel('q5')
    ax6.plot(t, q[5, :])
    ax6.set_ylabel('q6')
    ax6.set_xlabel('Time [sec]')
    plt.show()


def print_q_actual(q1, q2, q_actual, dt, headline):
    q = np.concatenate((q1, q2), axis=1)
    t_len = len(q_actual[0, :]) * dt
    t = np.linspace(0, t_len, len(q_actual[0, :]))

    f, (ax1, ax2, ax3, ax4, ax5, ax6) = plt.subplots(6, 1, sharex='col')
    if headline == 'angle':
        ax1.set_title('desired joints angle[rad]')
    elif headline == 'speed':
        ax1.set_title('desired joints speed[rad/sec]')

    ax1.plot(t, q[0, 0:len(q_actual[0, :])], t, q_actual[0, :])
    ax1.set_ylabel('q1')
    ax2.plot(t, q[1, 0:len(q_actual[0, :])], t, q_actual[1, :])
    ax2.set_ylabel('q2')
    ax3.plot(t, q[2, 0:len(q_actual[0, :])], t, q_actual[2, :])
    ax3.set_ylabel('q3')
    ax4.plot(t, q[3, 0:len(q_actual[0, :])], t, q_actual[3, :])
    ax4.set_ylabel('q4')
    ax5.plot(t, q[4, 0:len(q_actual[0, :])], t, q_actual[4, :])
    ax5.set_ylabel('q5')
    ax6.plot(t, q[5, 0:len(q_actual[0, :])], t, q_actual[5, :])
    ax6.set_ylabel('q6')
    ax6.set_xlabel('Time [sec]')
    plt.show()


def print_pos_error(q, dt):
    t_len = len(q[0, :]) * dt
    t = np.linspace(0, t_len, len(q[0, :]))
    f, (ax1, ax2, ax3, ax4, ax5, ax6) = plt.subplots(6, 1, sharex='col')
    ax1.set_title('Joints Position Error[rad]')
    ax1.plot(t, q[0, :])
    plt.grid(axis='both')
    ax1.set_ylabel('q1')
    ax2.plot(t, q[1, :])
    ax2.set_ylabel('q2')
    ax3.plot(t, q[2, :])
    ax3.set_ylabel('q3')
    ax4.plot(t, q[3, :])
    ax4.set_ylabel('q4')
    ax5.plot(t, q[4, :])
    ax5.set_ylabel('q5')
    ax6.plot(t, q[5, :])
    ax6.set_ylabel('q6')
    ax6.set_xlabel('Time [sec]')
    plt.show()


def print_force(f_in_log, dt):
    t_len = len(f_in_log[:]) * dt
    t = np.linspace(0, t_len, len(f_in_log[:]))
    plt.figure()
    plt.plot(t, f_in_log[:])
    plt.title('Force [N?]')
    plt.grid(axis='both')
    plt.ylabel('f_in')
    plt.xlabel('Time [sec]')
    plt.show()


def print_torque(u_log, dt):
    t_len = len(u_log[0, :]) * dt
    t = np.linspace(0, t_len, len(u_log[0, :]))
    f, (ax1, ax2, ax3, ax4, ax5, ax6) = plt.subplots(6, 1, sharex='col')
    ax1.set_title('Torque [Nm?]')
    ax1.plot(t, u_log[0, :])
    plt.grid(axis='both')
    ax1.set_ylabel('q1')
    ax2.plot(t, u_log[1, :])
    ax2.set_ylabel('q2')
    ax3.plot(t, u_log[2, :])
    ax3.set_ylabel('q3')
    ax4.plot(t, u_log[3, :])
    ax4.set_ylabel('q4')
    ax5.plot(t, u_log[4, :])
    ax5.set_ylabel('q5')
    ax6.plot(t, u_log[5, :])
    ax6.set_ylabel('q6')
    ax6.set_xlabel('Time [sec]')
    plt.show()


def clamp(num, min_value, max_value):
    return max(min(num, max_value), min_value)


def force_scope(sim, force_scope_log):
    forces = sim.data.cfrc_ext[sim.model.body_name2id('OUR_TABLE')]
    forces = np.reshape(forces, (6, 1))
    force_scope_log = np.append(force_scope_log, forces, axis=1)
    # filtered_force_log = my_filter.butter_lowpass_filter(force_scope_log, 5, 15, 2)  # 8, 20, 2
    return force_scope_log


def print_force_scope(f_log, dt):
    t_len = len(f_log[0, :]) * dt
    t = np.linspace(0, t_len, len(f_log[0, :]))
    f, (ax1, ax2, ax3, ax4, ax5, ax6) = plt.subplots(6, 1, sharex='col')
    ax1.set_title('Force and Torque')
    ax1.plot(t, f_log[0, :])
    plt.grid(axis='both')
    ax1.set_ylabel('Torque x')
    ax2.plot(t, f_log[1, :])
    ax2.set_ylabel('Torque y')
    ax3.plot(t, f_log[2, :])
    ax3.set_ylabel('Torque z')
    ax4.plot(t, f_log[3, :])
    ax4.set_ylabel('Force x')
    ax5.plot(t, f_log[4, :])
    ax5.set_ylabel('Force y')
    ax6.plot(t, f_log[5, :])
    ax6.set_ylabel('Force z')
    ax6.set_xlabel('Time [sec]')
    plt.show()


def print_force_scope2(force_log, dt):
    t_len = len(force_log[0, :]) * dt
    t = np.linspace(0, t_len, len(force_log[0, :]))
    f, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex='col')
    ax1.set_title('Force')
    ax1.plot(t, force_log[0, :])
    plt.grid(axis='both')
    ax1.set_ylabel('Force x')
    ax2.plot(t, force_log[1, :])
    ax2.set_ylabel('Force y')
    ax3.plot(t, force_log[2, :])
    ax3.set_ylabel('Force z')
    ax3.set_xlabel('Time [sec]')
    plt.show()


def contacts(sim):
    print('number of contacts', sim.data.ncon)
    for i in range(sim.data.ncon):
        # Note that the contact array has more than `ncon` entries,
        # so be careful to only read the valid entries.
        contact = sim.data.contact[i]
        print('contact', i)
        print('dist', contact.dist)
        print('geom1', contact.geom1, sim.model.geom_id2name(contact.geom1))
        print('geom2', contact.geom2, sim.model.geom_id2name(contact.geom2))
        # There's more stuff in the data structure
        # See the mujoco documentation for more info!
        geom2_body = sim.model.geom_bodyid[sim.data.contact[i].geom2]
        print('body: ', geom2_body)
        print(' Contact force on geom2 body', sim.data.cfrc_ext[geom2_body])
        print('norm', np.sqrt(np.sum(np.square(sim.data.cfrc_ext[geom2_body]))))
        # Use internal functions to read out mj_contactForce
        c_array = np.zeros(6, dtype=np.float64)
        print('c_array', c_array)
        mujoco_py.functions.mj_contactForce(sim.model, sim.data, i, c_array)
        print('c_array', c_array)
        print(' ')
    print('------------------------------------------------------- \n \n')


