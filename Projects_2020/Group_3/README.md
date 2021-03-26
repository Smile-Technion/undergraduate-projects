# Project-Wire Peg in a Hole 
# by: Gal Schachmundes and Nadav Amir

Hello,
We are two mechanical engneering graduates from the Technion - Israel Institute of Technology.
 1. Nadav Amir - ndvamr@gmail.com
 2. Gal Schachmundes - galsch911@gmail.com 

This git describes our undergraduate final project which we prepared during 2020-2021. 
feel free to contact us by email for any question.

# introduction
Our project was part of an association that aims to create a set of tools that will help industrial robots to behave better in situations that involve interaction with flexible objects and operation within close disance to human enviroment.
The activities are activities such as: assembling flexible components and wiring electrical cables.

# What was the purpose of our project?:
The purpose of the project was to simulate impedace behavior in a robotirc matipulator to performs an electrical cable wiring mission.

The project consists of:
 1.Impedance simulation in Matlab using Simscape multibody.
 2.Simulation built in mujoco py which simulate the operation of 3 DOF robot in a cable wiring mission: PD and impedance      controllers are avaliable. controllers and Impedance parameters can be adjusted.
 3.Reinforcement learning simulation built in mujoco py using gym enviroment and stable baselines DDPG algoritm. several      reward mechanisms has been tested.  


# Technical data - Installation of softwares:

applications to be installed:

-pycharm - https://www.jetbrains.com/pycharm/

-python 3.6 or later - https://www.python.org/downloads/

-matlab R2017b or  -https://www.mathworks.com/downloads/

-Mujoco 150 -http://www.mujoco.org/
    download from here: https://www.roboti.us/
    installation instructions (as we did it in our project):   
    1.Download mjpro150 win from the MuJoCo site.https://www.roboti.us/index.html.   
    2.Unzip the downloaded mujoco150 directory into ~/.mujoco/mujoco150, and place your license key (the mjkey.txt file         from your email) at ~/.mujoco/mjkey.txt.
        *If you want to specify a nonstandard location for the key and package, use the env variables MUJOCO_PY_MJKEY_PATH          and MUJOCO_PY_MUJOCO_PATH.
         testing Mujoco install correct: cd ~/.mujoco/mjpro150/bin/./simulate
        
 -mujocopy - https://github.com/openai/mujoco-py

-gym - https://gym.openai.com/docs/

-StableBaselines - https://stable-baselines.readthedocs.io/en/master/
end.
