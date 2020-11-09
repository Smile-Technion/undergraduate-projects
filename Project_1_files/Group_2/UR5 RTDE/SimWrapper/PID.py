import numpy as np
from SimWrapper.Controller import Controller
from SimWrapper.Enums import ControllerType


class PID(Controller):

    # def __init__(self):
    # def __iadd__(self, other):
    def __init__(self,simObj):
        Controller.__init__(self,simObj)
        self.Pgain = 0
        self.Igain = 0
        self.Dgain = 0
        self.e_integral = 0
        self.control_type = ControllerType.Regular
        self.EPgain = []

    def P(self, pgain):
        self.Pgain = pgain
        self.Igain = 0
        self.Dgain = 0
        m=3
        self.EPgain = [pgain,pgain,pgain,m*pgain,m*pgain,m*pgain]
        self.reset_integrator()

    def PI(self, pgain, igain):
        self.Pgain = pgain
        self.Igain = igain
        self.Dgain = 0

    def PD(self, pgain, dgain):
        self.Pgain = pgain
        self.Igain = 0
        self.Dgain = dgain

    def PID(self, pgain, igain, dgain):
        self.Pgain = pgain
        self.Igain = igain
        self.Dgain = dgain

    def get_u(self,q_des):
        self.calc_u(q_des)
        return self.u

    def calc_u(self,q_des):
        q_current = self.helper.get_current_joints()
        qvel = self.helper.get_current_joints_vel()
        e = q_des - q_current
        self.e_integral = self.e_integral + np.dot(self.dt, e)
        # self.u = np.dot(self.Pgain, e) + np.dot(self.Igain, self.e_integral) + np.dot(self.Dgain, qvel)
        self.u = np.multiply(self.EPgain, e) + np.dot(self.Igain, self.e_integral) + np.dot(self.Dgain, qvel)

    def reset_integrator(self):
        self.e_intergral = 0