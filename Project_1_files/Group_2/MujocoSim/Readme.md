MujocoSim contains all the files required to run the UR5 simulation ,

**Controller.py**  Parent class for control sub-class

**DataHandler.py** Creates 3D data object with simple log function

**Enums.py** Enumarated impedance control types

**Graph.py** plot data [desired position,impedance postion,postion error,control effort,contact force,desired trajectory]

**Impedance.py**  -solve impedance equations , using ODE 

**InverseKinematics.py**  -call matalab engine to preform inversekinematics to get rotation and joints angles.	

**PID.py**	- contain the functions which define the classic controller[P,PI,PID ] , and control effort

**RoboticsHelper.py**	-get data from mujoco - py simulation [inertia matrix,position,velocity,jacobian,find collision bodies]

**Simulation.py**	- define simulation initial state, camera position load xml model

**Task.py** - Go_to function which use closed loop control (regular or impedance),

**path calc** - build polynomial path one dimension.
![Simulation](https://github.com/SmileLab-technion/undergraduate-projects/blob/master/Project_1_files/Group_2/MujocoSim/image.png)

[![Watch Video](https://github.com/SmileLab-technion/undergraduate-projects/blob/master/Project_1_files/Group_2/images/youtube.png)](https://youtu.be/33m-gvbTRRs)

