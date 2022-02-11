import numpy as np
from scipy.spatial.transform import Rotation as R

#  this function takes the rotation axis and angle and converts them into the appropriate Rotation matrix.
#  for more info read : https://en.wikipedia.org/wiki/Rotation_matrix#Rotation_matrix_from_axis_and_angle
def R_axis_and_angle(a, theta):  # a is the axis and theta is given in radians
    c_theta = np.cos(theta)
    s_theta = np.sin(theta)
    v_theta = (1 - np.cos(theta))
    R = np.array([[c_theta + a[0] ** 2 * v_theta, a[0] * a[1] * v_theta - a[2] * s_theta,
                   a[0] * a[2] * v_theta + a[1] * s_theta],
                  [a[0] * a[1] * v_theta + a[2] * s_theta, c_theta + a[1] ** 2 * v_theta,
                   a[1] * a[2] * v_theta - a[0] * s_theta],
                  [a[0] * a[2] * v_theta - a[1] * s_theta, a[1] * a[2] * v_theta + a[0] * s_theta,
                   c_theta + a[2] ** 2 * v_theta]])
    return R


def mat2pos(Mat):  # given a position matrix, the function returns the vector:[x, y, z, Roll, Pitch, Yaw]
    pos_loc = Mat[0:3, 3]  # position
    rot_mat = Mat[0:3, 0:3]  # rotation Matrix
    rot = R.from_matrix(rot_mat)  # Scipy Rotation object
    pos_ang = rot.as_rotvec()  # Extracting orientation angles - Roll, Pitch, Yaw
    pos = np.append(pos_loc, pos_ang)
    return pos
