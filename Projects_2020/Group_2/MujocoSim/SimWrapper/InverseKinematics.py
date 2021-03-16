import numpy as np
import matlab.engine
import sys

class IK:

    def __init__(self):
        names = matlab.engine.find_matlab()
        print(names)
        if len(names) > 0:
            self.eng = matlab.engine.connect_matlab(names[0])
        else:
            self.eng = matlab.engine.start_matlab()
        self.eng.addpath(r'/home/mujoco/UR5/Sim/MATLAB_Scripts/ur',nargout=0)
        self.eng.addpath(r'/home/mujoco/UR5/Sim/MATLAB_Scripts/transformation',nargout=0)
        self.eng.cd(r'/home/mujoco/UR5/Sim/MATLAB_Scripts/ur')
        self.ikObj = self.eng.UR_Wrapper()

    def calcPos(self,q_current, pos, orn):
        q_temp = self.eng.calcJoints(self.ikObj, matlab.double(list(q_current)),matlab.double(list(pos)),matlab.double(list(orn)))
        return np.reshape(q_temp,(6))

