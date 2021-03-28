in order to open the simulations you need to do the following steps:
	
	A. install stable baselines.
	B. install gym.
	C. gym file replacement:
		1.gym\envs: replace __init__.py with file with __init__envs.py (after replacement - change the name for __init__.py)
		2.gym\envs\mujoco: replace __init__.py with file with __init__mujoco.py (after replacement - change the name for __init__.py)
		3. copy ProjectEnv.py, ProjectEnvIMP.py , ProjectEnvXYZ.py to gym\envs\mujoco. ProjectEnvImp is the current version of the enviroment.
		the other 2 files are previous versions.
		4.copy all the xml files and the mesh directory to: gym\envs\mujoco\assets.
		2dof__force_sensing_env_mechanismHorizontal3.xml is the current xml version of the project the other files are previous versions
	D.open new python project with the files as follows:
		1.Standard RL - open project with the files in the directory with the name "Standard RL"
	 	2.Impedance RL method 1   - open project with the files in the directory with the name "Impedance RL method 1"
		3.Impedance RL method 2   - open project with the files in the directory with the name "Impedance RL method 2"
	E. In order to adjuct learning hyperparameters such as learning rates, exploration , etc please do to: stable_baselines\ddpg\ddpg.py
