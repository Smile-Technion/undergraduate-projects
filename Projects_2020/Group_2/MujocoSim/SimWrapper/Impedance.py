import enum
import numpy as np
import scipy as sp
from Sim.SimWrapper.Controller import Controller
from Sim.SimWrapper.Enums import ControllerType, ImpedanceType
from Sim.SimWrapper.PID import PID

from scipy.integrate import odeint, ode
import control

class ImpedanceControl(PID):

    last_force = [0,0,0]
    last_v = [0,0,0]

    def __init__(self,simObj,type,k_imp, c_imp, m_imp):
        PID.__init__(self,simObj)
        self.type = type
        self.k = k_imp
        self.c = c_imp
        self.m = m_imp
        self.gripper = 'gripper_finger'
        self.control_type = ControllerType.Impedance
        self.ic_integral = []
        self.pos_imp = np.array([0,0,0],np.dtype(np.float64))
        self.vel_imp = np.array([0,0,0],np.dtype(np.float64))


    def get_imp(self,pos,orn,time):
        if self.type == ImpedanceType.IMIC:
            self.imic_logic(pos,orn)
        elif self.type.value == ImpedanceType.IMPB.value:
            self.impb_logic(pos,orn,time)
        elif self.type == ImpedanceType.IMDB:
            self.imdb_logic(pos,orn)
        return np.dot(1000,self.pos_imp)


    def impb_logic(self,pos,orn): #Not Implemented
        return 0

    def imic_logic(self,pos,orn,time):
        x = self.helper.get_gripper_xpos()-np.dot(pos, 1/1000)
        v = self.helper.get_gripper_vel()
        v = [0,0,0]
        force = self.helper.get_sensor_force()
        t0 = 0
        x0 = [float(x[0]), v[0]]
        y0 = [x[1], v[1]]
        z0 = [x[2], v[2]]
        Fx = force[0]
        Fy = force[1]
        Fz = force[2]
        t = [t0, t0+time]
        ximp = odeint(self.sim_model,x0, t, args=(Fx,))
        yimp = odeint(self.sim_model,y0, t, args=(Fy,))
        zimp = odeint(self.sim_model,z0, t, args=(Fz,))
        self.pos_imp = [ximp[1,0],yimp[1,0], zimp[1,0]]
        self.vel_imp = [ximp[1,1], yimp[1,1], zimp[1,1]]
        self.last_force = force


    def imdb_logic(self,pos,orn): #Not Implemented
        Jt = self.helper.get_JT(self.gripper);
        H = self.helper.get_H()
        tau = 0
        return 0

    #ODE model for impedance
    def sim_model(self,x0i, t0i, Fint):
        dxdt =  [x0i[1],(-self.c*x0i[1]-self.k*x0i[0]+Fint)/self.m]
        return dxdt


