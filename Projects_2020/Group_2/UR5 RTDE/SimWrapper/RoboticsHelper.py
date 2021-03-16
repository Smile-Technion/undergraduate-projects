import numpy as np
from SimWrapper.Simulation import Simulator
from SimWrapper.InverseKinematics  import IK
# from mujoco_py import (functions, MjSim, MjSimState)
class RHELPER:
    def __init__(self, ur5):
        self.ur5 = ur5
        self.last_forces_avg = [0,0,0]
        self.read_weight = 0.1

    def get_gripper_xpos(self):
        TCP_pos = self.ur5.state.actual_TCP_pose
        TCP_pos[2] = TCP_pos[2]
        return TCP_pos

    def get_gripper_vel(self):
        return self.ur5.state.actual_TCP_speed

    def get_current_joints(self):
        return self.ur5.state.actual_q
    def get_current_joints_vel(self):
        return self.ur5.state.actual_qd

    def get_sensor_force(self):
        return self.ur5.state.actual_TCP_force

    def get_gripper_orn(self):
        return self.ur5.state.actual_TCP_pose[3:6]







