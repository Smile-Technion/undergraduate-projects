
import numpy as np
# from SimWrapper.UR5_IK import inv_kin as ik
import threading
import sys
import matlab.engine
class IK:
    global q_temp
    def __init__(self):
        self.ikObj = []


    def calcPos(self,q_cur,pos_current,rot_current, pos_target,rot_target):
        q_des = self.ikObj.inverse_kinematics(pos_target,rot_target,q_cur)
        return q_des
        # np.reshape(q_temp,(6))

    def Rx(self,theta):
        H = np.zeros((4,4))
        H[1,1] = np.cos(theta)
        H[1,2] = -np.sin(theta)
        H[2,1] = np.sin(theta)
        H[2,2] = np.cos(theta)
        H[0,0] = 1
        H[3,3] = 1
        return H

    def Ry(self, theta):
        H = np.zeros((4,4))
        H[0,0] = np.cos(theta)
        H[0,2] = np.sin(theta)
        H[2,0] = -np.sin(theta)
        H[2,2] = np.cos(theta)
        H[1,1] = 1
        H[3,3] = 1
        return H

    def Rz(self, theta):
        H = np.zeros((4,4))
        H[0,0] = np.cos(theta)
        H[0,1] = -np.sin(theta)
        H[1,0] = np.sin(theta)
        H[1,1] = np.cos(theta)
        H[2,2] = 1
        H[3,3] = 1
        return H
