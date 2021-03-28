import numpy as np
from gym import utils
from mujoco_py import (functions)
from gym.envs.mujoco import mujoco_env

class ProjectEnv(mujoco_env.MujocoEnv, utils.EzPickle):
    def __init__(self):
        utils.EzPickle.__init__(self)
        mujoco_env.MujocoEnv.__init__(self, '2dof__force_sensing_env_mechanismHorizontal3.xml', 2)

    def step(self, a):
        vec = self.get_body_com("fingertip")-self.get_body_com("target")
        #reward_dist = - np.linalg.norm(vec)
        if np.linalg.norm(vec)>=0.02:
            reward_dist = - np.linalg.norm(vec)
        if np.linalg.norm(vec)<0.02:
            reward_dist = + 0.8
        if np.linalg.norm(a)>=0.05:
            reward_ctrl = - np.linalg.norm(a)
        if np.linalg.norm(a)<0.05:
            reward_ctrl = + 0.4
        #reward_ctrl = - np.square(a).sum()
        #reward_ctrl = 0
        force,torque= self.getForces(3, 'fingertip')
        Forces = np.array([force[0], force[1], force[2], torque[0], torque[1], torque[2]]).reshape(-1, 1)
        if np.linalg.norm([Forces[0],Forces[1],Forces[2]])>400000:
            reward_force = - np.linalg.norm([Forces[0],Forces[1],Forces[2]])/10000000
        if np.linalg.norm([Forces[0],Forces[1],Forces[2]])<400000:
            reward_force = + 0.4
        #reward_force =0
        if (self.sim.data.qpos[0] + self.sim.data.qpos[1] + self.sim.data.qpos[2]) < np.pi / 2:
            reward_angle = -(np.pi / 2 - self.sim.data.qpos[0] - self.sim.data.qpos[1] - self.sim.data.qpos[2]) / 10
        if (self.sim.data.qpos[0] + self.sim.data.qpos[1] + self.sim.data.qpos[2]) >= np.pi / 2:
            reward_angle = +(np.pi / 2 - self.sim.data.qpos[0] - self.sim.data.qpos[1] - self.sim.data.qpos[2]) / 10
        if np.abs((np.pi/2 - np.abs(self.sim.data.qpos[0] + self.sim.data.qpos[1] + self.sim.data.qpos[2])))<0.02:
            reward_angle = +0.5

        #if np.linalg.norm(force) < 1:
        #    reward_force = - np.linalg.norm(force)
        #if ((np.linalg.norm(force) > 1) and (np.linalg.norm(force) <= 10)):
        #    reward_force = - np.linalg.norm(force)/10
        #if ((np.linalg.norm(force) > 10) and (np.linalg.norm(force) <= 100)):
        #    reward_force = - np.linalg.norm(force) / 100
        #if ((np.linalg.norm(force) > 100) and (np.linalg.norm(force) <= 1000)):
        #    reward_force = - np.linalg.norm(force) / 1000
        #if ((np.linalg.norm(force) > 1000) and (np.linalg.norm(force) <= 10000)):
        #    reward_force = - np.linalg.norm(force) / 10000
        #if ((np.linalg.norm(force) > 10000) and (np.linalg.norm(force) <= 100000)):
        #    reward_force = - np.linalg.norm(force) / 100000
        #reward_force = 0
        reward = reward_dist + 1*reward_ctrl + 1*reward_force +reward_angle
        self.do_simulation(a, self.frame_skip)
        ob = self._get_obs()
        done = False
        return ob, reward, done, dict(reward_dist=reward_dist, reward_ctrl=reward_ctrl, reward_force=reward_force, reward_angle=reward_angle)

    def viewer_setup(self):
        self.viewer.cam.trackbodyid = 0

    def reset_model(self):
        qpos = self.np_random.uniform(low=0, high=np.pi, size=self.model.nq) + self.init_qpos
        while True:
            self.goal = np.array([0.0, 0.26])
            #self.goal = self.np_random.uniform(low=-.2, high=.2, size=2)
            if np.linalg.norm(self.goal) < 0.4:
                break
        qpos[3:5] = self.goal
        qpos[5] = -0.008
        qpos[6] = 0.008
        qvel = self.init_qvel + self.np_random.uniform(low=-.005, high=.005, size=self.model.nv)
        qvel[3:5] = 0
        qvel[5:7] = 0
        self.set_state(qpos, qvel)
        return self._get_obs()

    def _get_obs(self):
        theta = self.sim.data.qpos.flat[:3]
        return np.concatenate([
            np.cos(theta),
            np.sin(theta),
            self.sim.data.qpos.flat[3:],
            self.sim.data.qvel.flat[:3],
            self.get_body_com("fingertip") - self.get_body_com("target")
        ])

    def getForces(self, site_id, bodyname):
        force=[0,0,0]
        torque=[0,0,0]
        oritem=np.zeros([3,3], dtype=np.float64)
        ctemp = np.zeros(6, dtype=np.float64)
        csum = np.zeros(6, dtype=np.float64)
        if self.sim.data.ncon>0: #number of contacts in the simulation
            for i in range(self.sim.data.ncon):
                contact = self.sim.data.contact[i]
                if contact.geom1 == 4 or contact.geom2 == 4: #depends of how many links
                    oritemp=np.array(self.sim.data.contact[i].frame).reshape(3,3)
                    functions.mj_contactForce(self.sim.model,self.sim.data, i, ctemp)
                    ctemp[0:3]=np.dot(np.linalg.inv(oritemp),np.array(ctemp[0:3]))
                    ctemp[3:6] = np.dot(np.linalg.inv(oritemp),np.array(ctemp[3:6]))
                    csum+=ctemp
            force  = [-csum[0],-csum[1],-csum[2]]
            #torque = csum[3:6] #signs are incorrect
            torque=[-self.sim.data.sensordata[9],-self.sim.data.sensordata[10],-self.sim.data.sensordata[11]]
            #force=-np.array(force)
            #torque=-np.array(torque)
            torque=[torque[0],torque[1],0]
        if self.sim.data.ncon==0:
            force=[0,0,0]
            torque=[0,0,0]
        return -np.array(force), -np.array(torque)