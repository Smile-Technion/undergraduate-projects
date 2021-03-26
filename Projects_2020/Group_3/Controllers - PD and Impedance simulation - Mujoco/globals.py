from mujoco_py import (MjSim)
from mujoco_py_renderer import MujocoPyRenderer
from mujoco_py import (load_model_from_path)

global scenario
scenario=1
#The following diementions of the robot arms need to be consistent with the XML model:
global L1
L1 = 1  # in meter
global L2
L2 = 1.2 # in meter
global L3
L3 = 0.4  # in meter
global SimStep # default simulation step.
SimStep = 0.002  # in  - mujoco default.
global waitingTime # waiting time before the simulation starts.
waitingTime = 2 #in seconds

if scenario==1:
    #Here you need to extract the path to the XML file of your model.
    model = load_model_from_path("C:\\Users\\nadavamir\\OneDrive - Technion\\Final Project\\rl_part\\IMPCHECK2\\2dof__force_sensing_env_mechanismHorizontal2.xml")
sim = MjSim(model)
viewer = MujocoPyRenderer(sim)

