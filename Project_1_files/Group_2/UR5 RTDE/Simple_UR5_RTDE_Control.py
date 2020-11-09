import random

import numpy as np
import time
from SimWrapper.Enums import ControllerType
from SimWrapper.Graph import Graph
from SimWrapper.Impedance import ImpedanceControl
from SimWrapper.PID import PID
from SimWrapper.Simulation import Simulator
from SimWrapper.Task  import Task
from SimWrapper.Controller  import Controller
from SimWrapper.Enums  import ImpedanceType
from SimWrapper.RTDEControl  import RTDECon



t = time.time()
ur5 = RTDECon()

controller = PID(ur5)
controller.P(1)

M = 25
wn = 5
zeta = 8
K = M*(wn**2)
C = 2*M*wn*zeta
print(K)
print(C)

Mt = 3
wnt = 2
zetat = 2
Kt = Mt*(wnt**2)
Ct = 2*Mt*wnt*zetat

controller = ImpedanceControl(ur5,ImpedanceType.IMPB, K,C,M,Kt,Ct,Mt)
controller.P(10)
task = Task(ur5,controller)
task.debug = True

orn = [np.pi,0,0]
pos = [0.2570,-0.230,0.1]

p_start = [0.040, -0.571, 0.018]
p_end = [0.040,-0.590,0.009]
p_1 = [0.08, -0.580, 0.029]
p_2 = [0.08, -0.580, 0.009]
p_3 = [0.12, -0.580, 0.029]
p_4 = [0.12, -0.580, 0.009]
p_5 = [0.15, -0.580, 0.029]
p_6 = [0.15, -0.580, 0.009]


task.goToByPath(p_start,orn,T_end = 2000,step_time=2)
print("new path")
task.goToByPath(p_end,orn,T_end = 2000,step_time=2)
task.goToByPath(p_1,orn,T_end = 2000,step_time=2)
task.goToByPath(p_2,orn,T_end = 2000,step_time=2)
task.goToByPath(p_3,orn,T_end = 2000,step_time=2)
task.goToByPath(p_4,orn,T_end = 2000,step_time=2)
task.goToByPath(p_5,orn,T_end = 2000,step_time=2)
task.goToByPath(p_6,orn,T_end = 2000,step_time=2)

print("end")
