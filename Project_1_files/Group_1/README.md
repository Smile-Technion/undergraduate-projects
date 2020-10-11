# Peg in a Hole(Yamit and Saar):
The project is part of an association that aims to create a set of tools that will help industrial robots deal with a variety of functions, , such as: assembling flexible components ( for e.g. rubber bands) , and threading electrical wires.
The purpose of our project is to create a simulation in the mujoco software of a 6D robot that: performs a Peg-in-Hole mission using impedance control.
The project includes: Examining the tolerances of accuracy and success of the task, and proving that impedance control improves success rates.

# Parts of the project:
1. Simulation of a 2-degree freedom robot with PID and IM-IC control
2. Simulation of UR5 robot in mujoco software
3. Solve the "inverse kinematics" problem for the UR5 robot
4. Combination of PID and IM-IC controllers within the simulation
5. Improving mission success by spiraling search

# Functions:
UR5_kinematics.py - This function has the solution of the "inverse kinematics" and "forward kinematics" problem of the UR5 robot using the H-D method

# Mujoco_Sim:
Contains the model of the problem in Mujoco, And the full simulation "main.py"

# Docs:
Contains all presentations, articles, summaries and reports of the project.

# Multimedia:
Contains the photos, graphs and videos of the project
