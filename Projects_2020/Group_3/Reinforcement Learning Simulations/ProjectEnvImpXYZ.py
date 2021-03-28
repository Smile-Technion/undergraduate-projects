import numpy as np
from gym import utils
from mujoco_py import (functions)
from gym.envs.mujoco import mujoco_env
from scipy.integrate import odeint


class ProjectEnvImpXYZ(mujoco_env.MujocoEnv, utils.EzPickle):
    def __init__(self):
        utils.EzPickle.__init__(self)
        mujoco_env.MujocoEnv.__init__(self, '2dof__force_sensing_env_mechanismHorizontal3.xml', 2)

    def step(self, a):
        vec = self.get_body_com("fingertip")-self.get_body_com("target")
        if np.linalg.norm(vec)>=0.02:
            reward_dist = - np.linalg.norm(vec)
        if np.linalg.norm(vec)<0.02:
            reward_dist = + 0.4

        if np.linalg.norm(a)>=0.05:
            reward_ctrl = - np.linalg.norm(a)
        if np.linalg.norm(a)<0.05:
            reward_ctrl = + 0.4

        #reward_ctrl = 0
        force,torque= self.getForces(3, 'fingertip')
        Forces = np.array([force[0], force[1], force[2], torque[0], torque[1], torque[2]]).reshape(-1, 1)
        ForcesImp = (np.array([Forces[0], Forces[1],Forces[2]])).reshape(1,-1)


        if (self.sim.data.qpos[0] + self.sim.data.qpos[1] + self.sim.data.qpos[2]) < np.pi / 2:
            reward_angle = -(np.pi / 2 - self.sim.data.qpos[0] - self.sim.data.qpos[1] - self.sim.data.qpos[2]) / 10
        if (self.sim.data.qpos[0] + self.sim.data.qpos[1] + self.sim.data.qpos[2]) >= np.pi / 2:
            reward_angle = +(np.pi / 2 - self.sim.data.qpos[0] - self.sim.data.qpos[1] - self.sim.data.qpos[2]) / 10
        if np.abs((np.pi/2 - np.abs(self.sim.data.qpos[0] + self.sim.data.qpos[1] + self.sim.data.qpos[2])))<0.02:
            reward_angle = +0.4

        # fingertip = self.get_body_com("fingertip")
        # x0 = np.array([fingertip[0],fingertip[1],fingertip[2]]).reshape(-1,1)
        # x0_vel=np.array([self.sim.data.sensordata[0], self.sim.data.sensordata[1], self.sim.data.sensordata[2]]).reshape(-1,1)
        # if(self.sim.data.time==0): #first iteration before reset
        #     self.xm = x0
        #     self.xmDot = np.zeros(3).reshape(-1,1)
        #     self.xmDotaim = np.zeros(3)
        #
        # if(self.sim.data.time>0):
        #     [self.xm, self.xmDot, self.xmDotaim] = self.Impedance_ode(self.xm, self.xmDot, self.K, self.B, self.M, x0, x0_vel, ForcesImp)
        # impedance_vec = x0 - self.xm
        # impedance_velocity_vec = x0_vel - self.xmDot
        # reward_impedance = - np.linalg.norm(impedance_vec)
        reward_impedance = 0

        force, torque = self.getForces(3, 'fingertip')
        Forces = np.array([force[0], force[1], force[2], torque[0], torque[1], torque[2]]).reshape(-1, 1)

        reward_force = - np.linalg.norm([Forces[0], Forces[1], Forces[2]]) / 10000000


        reward = reward_dist + reward_ctrl + reward_angle + reward_impedance*0 + reward_force
        self.do_simulation(a, self.frame_skip)
        ob = self._get_obs()
        done = False
        return ob, reward, done, dict(reward_dist=reward_dist, reward_ctrl=reward_ctrl, reward_angle=reward_angle, reward_impedance=reward_impedance, reward_force=reward_force)

    def viewer_setup(self):
        self.viewer.cam.trackbodyid = 0

    def reset_model(self):
        qpos = self.np_random.uniform(low=0, high=np.pi, size=self.model.nq) + self.init_qpos
        self.xm = np.array([self.get_body_com("fingertip")[0],self.get_body_com("fingertip")[1], (self.sim.data.qpos[0] + self.sim.data.qpos[1] + self.sim.data.qpos[2])]).reshape(-1,1)
        self.xmDot = np.zeros(3).reshape(-1,1)
        self.k = 100
        k = self.k
        self.b = 10
        b = self.b
        self.m = 10
        m = self.m

        self.K = np.array([[k, 0, 0],
                          [0, k, 0],
                          [0, 0, k]])
        self.M = np.array([[m, 0, 0],
                          [0, m, 0],
                          [0, 0, m]])
        self.B = np.array([[b, 0, 0],
                          [0, b, 0],
                          [0, 0, b]])
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

    def getForces(self,site_id, bodyname):
        force = [0, 0, 0]
        torque = [0, 0, 0]
        oritem = np.zeros([3, 3], dtype=np.float64)
        ctemp = np.zeros(6, dtype=np.float64)
        csum = np.zeros(6, dtype=np.float64)
        if self.sim.data.ncon > 0:  # number of contacts in the simulation
            for i in range(self.sim.data.ncon):
                contact = self.sim.data.contact[i]
                if contact.geom1 == 8 or contact.geom2 == 8:  # depends of how many links
                    oritemp = np.array(self.sim.data.contact[i].frame).reshape(3, 3)
                    functions.mj_contactForce(self.sim.model, self.sim.data, i, ctemp)
                    ctemp[0:3] = np.dot(np.linalg.inv(oritemp), np.array(ctemp[0:3]))
                    ctemp[3:6] = np.dot(np.linalg.inv(oritemp), np.array(ctemp[3:6]))
                    csum += ctemp
            force = [-csum[0], -csum[1], -csum[2]]
            # torque = csum[3:6] #signs are incorrect
            torque = [-self.sim.data.sensordata[9], -self.sim.data.sensordata[10], -self.sim.data.sensordata[11]]
        if self.sim.data.ncon == 0:
            force = [0, 0, 0]
            torque = [0, 0, 0]
        return force, torque

    def impedance_differential_equation(self, y, t, K, B, M, x0, x0dot, Fint):
        x1, y1, theta1, x2, y2, theta2 = y
        xm = [x1, y1, theta1]
        xm = np.array(xm).reshape(-1, 1)
        xmdot = [x2, y2, theta2]
        xmdot = np.array(xmdot).reshape(-1, 1)
        # Fint = 10*np.array(Fint)
        dydt = [xmdot, (np.matmul((np.matmul(K, np.linalg.inv(M))), x0)).reshape(-1, 1) + \
                (np.matmul((np.matmul(B, np.linalg.inv(M))), x0dot)).reshape(-1, 1) - \
                (np.matmul(Fint, np.linalg.inv(M))).reshape(-1, 1) - \
                np.matmul((np.matmul(K, np.linalg.inv(M))), xm) - \
                np.matmul((np.matmul(B, np.linalg.inv(M))), xmdot)]
        dydt1 = np.array(dydt[0])
        dydt2 = np.array(dydt[1])
        dydt = np.append(dydt1, dydt2)
        return dydt

    def Impedance_ode(self, xm, xmDot, K, B, M, x0, x0dot, Fint):
        xm0 = [xm[0,0], xm[1,0], xm[2,0], xmDot[0,0], xmDot[1,0], xmDot[2,0]]
        sol = odeint(self.impedance_differential_equation, xm0, [0, 0.01], args=(K, B, M, x0, x0dot, Fint))
        sol1 = [sol[1, 0], sol[1, 1], sol[1, 2], sol[1, 3], sol[1, 4], sol[1, 5]]
        sol1 = np.array(sol1)
        xm = [sol1[0], sol1[1], sol1[2]]
        xm2 = np.array(xm).reshape(-1, 1)
        xmdot = [sol1[3], sol1[4], sol1[5]]
        xmdot2 = np.array(xmdot).reshape(-1, 1)
        # Fint = 10 * np.array(Fint)
        xmdotaim2 = (np.matmul((np.matmul(K, np.linalg.inv(M))), x0)).reshape(-1, 1) + \
                    (np.matmul((np.matmul(B, np.linalg.inv(M))), x0dot)).reshape(-1, 1) - \
                    (np.matmul(Fint, np.linalg.inv(M))).reshape(-1, 1) - \
                    np.matmul((np.matmul(K, np.linalg.inv(M))), xm2) - \
                    np.matmul((np.matmul(B, np.linalg.inv(M))), xmdot2)
        return xm2, xmdot2, xmdotaim2
