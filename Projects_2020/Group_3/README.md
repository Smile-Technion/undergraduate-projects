# Project-Wire Peg in a Hole 
# by: Gal Schachmundes and Nadav Amir

Hello,
We are two mechnical engneering graduates from the Technion - Israel Institute of Technology.
1. Nadav Amir - ndvamr@gmail.com
2. Gal Schachmundes - galsch911@gmail.com 

This git describes our undergraduate final project which we prepared during 2020-2021. 
feel free to contact us by email for any question.

Our project was part of an association that aims to create a set of tools that will help industrial robots to behave better in situations that involve interaction with flexible objects and operation within close disance to humans .
The activities are activities such as: assembling flexible components and wiring electrical wires.

# What was the purpose of our project?:
The purpose of the project was to simulate impedace behavior in a robotirc  matipulator to performs an electrical cable wiring mission.
The project consists of:

1.Impedance simulation in matlab - simscape.
2.Simulation built in mujoco py which simulate the operation of 3 DOF robot in a cable wiring mission: PD and impedance controllers are avaliable.
3.Reinforcement learning simulation built in mujoco py using gym enviroment and stable baselines DDPG algoritm.  

# Parts of the project:
1. Simulation of a 2-degree freedom robot with PID and IM-IC control
2. Simulation of UR5 robot in mujoco software
3. Solve the "inverse kinematics" problem for the UR5 robot
4. Combination of PID and IM-IC controllers within the simulation
5. Improving mission success by spiraling search

# 2D Robot Simulation:
Simulation of a 2-degree freedom robot with PID and IM-IC control, the simulation was written in Matlab

# Functions:
UR5_kinematics.py - This function has the solution of the "inverse kinematics" and "forward kinematics" problem of the UR5 robot using the H-D method

# Mujoco Simulation:
Contains the model of the problem in Mujoco, And the full simulation "main.py" , simulation with the spiral search called "main_with_spiral.py"

# Docs:
Contains all presentations, articles, summaries and reports of the project.

# Multimedia:
Contains the photos, graphs and videos of the project
