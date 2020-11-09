import numpy as np
from numpy import linalg


import cmath
import math
from math import cos as cos
from math import sin as sin
from math import atan2 as atan2
from math import acos as acos
from math import asin as asin
from math import sqrt as sqrt
from math import pi as pi


class URkinematics(object):

    def __init__(self,ur_version="ur5",gripper=False,gripper_size=0):
        if ur_version=="ur5":
            self.d1 = 0.1625
            self.a2 = -0.425
            self.a3 = -0.39225
            self.d4 = 0.1333
            self.d5 = 0.0997
            self.d6 = 0.0996
            self.alpha=np.array([0,math.pi/2, 0, 0, math.pi/2, -math.pi/2, 0 ])
            self.theta = np.array([0, 0, 0, 0, 0, 0])

        elif ur_version=="ur10":
            self.d1 = 0.1273
            self.a2 = -0.612
            self.a3 = -0.5723
            self.d4 = 0.163941
            self.d5 = 0.1157
            self.d6 = 0.0922
            self.alpha = np.array([pi / 2, 0, 0, pi / 2, -pi / 2, 0])
            self.theta= np.array([0, 0, 0, 0, 0, 0])

        else:
            assert ("{} Not a valid value".format(ur_version))

        self.d=np.array([self.d1,0,0,self.d4,self.d5,self.d6],dtype=np.float32)
        self.a=np.array([0,0,self.a2,self.a3,0,0,0],dtype=np.float32)
        if gripper==True:
            self.d[-1]+=gripper_size
            self.d6+=gripper_size

    # ************************************************** FORWARD KINEMATICS

    def T_i_i_1(self,theta,num):
        num=num-1
        a=self.a[num]
        alpha= self.alpha[num]
        d=self.d[num]

        T=np.array([
            [cos(theta),-sin(theta),0,a],
            [sin(theta)*cos(alpha),cos(theta)*cos(alpha),-sin(alpha),-sin(alpha)*d],
            [sin(theta)*sin(alpha),cos(theta)*sin(alpha),cos(alpha),cos(alpha)*d],
            [0,0,0,1]
        ])
        return T

    def forward_kinematics(self,theta,f=6):
        all=[]
        for i in range(f):
            all.append(self.T_i_i_1(theta[i],i+1))

        return np.linalg.multi_dot(all)

    def euler_to_transformation_matrix(self,pos,angle):
        tran_matrix=np.identity((4),dtype=np.float32)
        tran_matrix[:3,3]=pos
        alpha,beta,gamma=angle

        MatRx = np.array([[1, 0, 0],
                          [0, cos(alpha), -sin(alpha)],
                          [0, sin(alpha), cos(alpha)]])
        MatRy = np.array([[cos(beta), 0, sin(beta)],
                          [0, 1, 0],
                          [-sin(beta), 0, cos(beta)]])
        MatRz = np.array([[cos(gamma), -sin(gamma), 0],
                          [sin(gamma), cos(gamma), 0],
                          [0, 0, 1]])
        # ZYX Euler
        tran_matrix[:3,:3] = np.dot(MatRz, np.dot(MatRy, MatRx))
        return tran_matrix


    # ************************************************** INVERSE KINEMATICS



    def inverse_kinematics(self,desired_pos,desired_orientation,current_theta):

         current_theta=current_theta
         T_0_6=self.euler_to_transformation_matrix(desired_pos,desired_orientation)
         #-----------------x-y are opposite then in simulation compare to the paper---------------------------------------
         # T is the full transformation matrix, T_0_6 for example is the 6 link viewed from coordinate system in 0.
         T_0_6[0,:]=-T_0_6[0,:]
         T_0_6[1, :] = -T_0_6[1, :]

         X_0_6=T_0_6[:3,0]
         Y_0_6=T_0_6[:3,1]
         Z_0_6=T_0_6[:3,2]
         P_0_6=T_0_6[:3,3]

         # ******************************** theta1 ****************************************

         P_05 = np.dot(T_0_6 , np.array([0, 0, -self.d6, 1]).T )
         #P_05 = (desired_pos * mat([0, 0, -self.d6, 1]).T - mat([0, 0, 0, 1]).T)
         current_theta_1 = current_theta[0]
         phi_1 = atan2(P_05[1], P_05[0])
         phi_2 = acos(self.d4 / sqrt(P_05[0] **2 + P_05[1] **2))
         theta_1=np.array([pi / 2 + phi_1 + phi_2,pi / 2 + phi_1 - phi_2,pi / 2 + phi_1 + phi_2-2*np.pi,pi / 2 + phi_1 - phi_2-2*np.pi])

         #theta_T_return1_2 = pi / 2 + phi_1 - phi_2
         theta_1=theta_1[np.argmin(np.abs(theta_1-current_theta_1))]

         # ******************************* theta5 *********************************************
         current_theta_5 = current_theta[4]
         theta_5=acos((P_0_6[0]*sin(theta_1)-P_0_6[1]*cos(theta_1)-self.d4)/(self.d6))
         theta_5=np.array([theta_5,-theta_5,theta_5-2*np.pi,-theta_5-2*np.pi])
         theta_5 = theta_5[np.argmin(np.abs(theta_5 - current_theta_5))]


         # ************************************ theta_6 *********************************************

         theta_6=atan2((-X_0_6[1]*sin(theta_1)+Y_0_6[1]*cos(theta_1))/(sin(theta_5+(theta_5==0)*1e-6)),
                       (X_0_6[0]*sin(theta_1)-Y_0_6[0]*cos(theta_1))/(sin(theta_5+(theta_5==0)*1e-6)))
         # **** theta3 ****
         T_0_1=self.T_i_i_1(theta_1,1)
         T_4_5 = self.T_i_i_1(theta_5, 5)
         T_5_6 = self.T_i_i_1(theta_6, 6)
         T_1_6=np.dot(np.linalg.inv(T_0_1),T_0_6)
         T_1_5=np.dot(T_1_6,np.linalg.inv(T_5_6))
         T_1_4=np.dot(T_1_5,np.linalg.inv(T_4_5))

         P_1_4=T_1_4[:3,3]

         P_4_xz=P_1_4[0]**2+P_1_4[2]**2

         # ************************************ theta_3 *********************************************
         current_theta_3 = current_theta[2]
         theta_3=acos((P_4_xz-self.a2**2-self.a3**2)/(2*self.a2*self.a3))
         theta_3=np.array([-theta_3,theta_3,-theta_3-2*np.pi,theta_3-2*np.pi])
         theta_3 = theta_3[np.argmin(np.abs(theta_3 - current_theta_3))]


         # ************************************ theta_2 *********************************************
         theta_2=atan2(-P_1_4[2],-P_1_4[0])-asin(-self.a3*sin(theta_3)/(np.abs(np.sqrt(P_4_xz))))
         # **** theta4 ****
         T_1_2 = self.T_i_i_1(theta_2, 2)
         T_2_3 = self.T_i_i_1(theta_3, 3)
         T_2_4 = np.dot(np.linalg.inv(T_1_2), T_1_4)
         T_3_4=  np.dot(np.linalg.inv(T_2_3), T_2_4)
         X_x=T_3_4[0,0]
         X_y=T_3_4[1,0]

         theta_4=atan2(X_y,X_x)
         Theta=np.array([theta_1,theta_2,theta_3,theta_4,theta_5,theta_6])
         #check=np.max(np.abs(Theta-current_theta))
         #location=np.argmax(np.abs(Theta-current_theta))

         #print(check*180/np.pi,location)
         #print(check * 180 / np.pi, location)
         return Theta