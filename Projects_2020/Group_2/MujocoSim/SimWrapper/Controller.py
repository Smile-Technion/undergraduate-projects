
import numpy as np
from SimWrapper.Simulation import Simulator
from SimWrapper.InverseKinematics  import IK
from SimWrapper.RoboticsHelper import RHELPER

class Controller:

    def __init__(self,simObj):
        self.u = []
        self.sim = simObj.sim
        self.dt = self.sim.model.opt.timestep
        self.helper = RHELPER(self.sim)
        self.control_type = []
    def get_u(self):
        return self.u

    def calc_u(self):
        return
