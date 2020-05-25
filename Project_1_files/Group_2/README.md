## Project-elastic band Peg in a Hole(Nevo and Guy)

**applications to be installed:**

  pycharm
  
  python 3.6
  
  matlab R2014b or later
  
  Mujoco 200
  
  mujoco-py**
  

**Steps for installing and setup
Download Mujoco200:**

  1.Download mjpro200 linux from the MuJoCo site.https://www.roboti.us/index.html
  
  2.Unzip the downloaded mujoco200 directory into ~/.mujoco/mujoco200, and place your license key (the mjkey.txt file from your email) at       ~/.mujoco/mjkey.txt.
  
  *If you want to specify a nonstandard location for the key and package, use the env variables MUJOCO_PY_MJKEY_PATH and MUJOCO_PY_MUJOCO_PATH.
  
  testing Mujoco install correct:
  cd ~/.mujoco/mjpro150/bin/./simulate
  
**Pycharm:**

  install pycharm -https://www.jetbrains.com/pycharm/
  
**Mujoco-py:

  -Linux with Python 3.6+. See the Dockerfile for the canonical list of system dependencies.https://www.python.org/downloads/
  -OS X with Python 3.6+.
  install mujoco -py
  copy this line to the bashrc file.
  ~/.bashrc
  "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/(username)/.mujoco/mjpro200/bin"
   source ~/.bashrc
   -cd
   git clone https://github.com/openai/mujoco-py.git

   And then paste the lines below into your command window:
   ```
  cd mujoco-py/
  
  sudo apt-get update
  
  sudo apt-get install patchelf
  
  sudo apt-get install python3 python-dev python3-dev build-essential libssl-dev libffi-dev libxml2-dev libxslt1-dev zlib1g-dev    
  libglew1.5 libglew-dev python-pip
  
  sudo apt-get install libgl1-mesa-dev libgl1-mesa-glx libosmesa6-dev python3-pip python3-numpy python3-scipy 
  
  sudo pip3 install -r requirements.txt
  
  sudo pip3 install -r requirements.dev.txt
  
  sudo python3 setup.py install
  
  sudo pip3 install gym
```
  test  :
  
  import gym
  env = gym.make('FetchPush-v1')
  env.reset()
  for _ in range(1000):
    env.render()
    env.step(env.action_space.sample()) # take a random action
    
troubleshooting installing mujoco-py ?\
use the following link git https://github.com/openai/mujoco-py\
 **Matlab engine for python:**
\install matlab engine for python : 
   follow the instructions on this link -
   https://www.mathworks.com/help/matlab/matlab_external/install-the-matlab-engine-for-python.html
   following link should be helpful for problames during matlab engine install
   https://stackoverflow.com/questions/54851026/how-to-install-matlab-engine-api-for-python-with-python-3-7
   
   **change the path in InverseKinematics.py to you'r own project path relative to 'ur' and transformation folders**
    
    
    
    
  
  
