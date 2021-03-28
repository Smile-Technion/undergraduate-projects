import numpy as np
from gym import utils
from mujoco_py import (functions)
from gym.envs.mujoco import mujoco_env
from scipy.integrate import odeint
from numpy import savetxt


class ProjectEnvImp(mujoco_env.MujocoEnv, utils.EzPickle):
    def __init__(self):
        utils.EzPickle.__init__(self)
        mujoco_env.MujocoEnv.__init__(self, '2dof__force_sensing_env_mechanismHorizontal3.xml', 2)

    def step(self, a):
        # POSITION PRICE
        fingertip = self.get_body_com("fingertip")
        vec = fingertip - self.get_body_com("target")
        if np.linalg.norm(vec) >= 0.03:
            reward_dist = - np.linalg.norm(vec)
        if np.linalg.norm(vec) < 0.03:
            reward_dist = 1 - 10*np.linalg.norm(vec)


        # CONTROL PRICE
        # not corverge very well but actually do minimize control effort:
       # if  np.square(a).sum()>2:
       #         reward_ctrl = - np.square(a).sum()/10 + 0.2
       # if  np.square(a).sum()<2:
       #         reward_ctrl =  0.2 - np.square(a).sum()/10
       # if  np.square(a).sum() < 1:
       #         reward_ctrl = reward_ctrl + 0.1
       # print(np.square(a).sum())
        reward_ctrl=0
        # ANGLE REWARD

        if (self.sim.data.qpos[0] + self.sim.data.qpos[1] + self.sim.data.qpos[2]) < np.pi / 2:
            reward_angle = -(np.pi / 2 - self.sim.data.qpos[0] - self.sim.data.qpos[1] - self.sim.data.qpos[2]) / 10
        if (self.sim.data.qpos[0] + self.sim.data.qpos[1] + self.sim.data.qpos[2]) >= np.pi / 2:
            reward_angle = +(np.pi / 2 - self.sim.data.qpos[0] - self.sim.data.qpos[1] - self.sim.data.qpos[2]) / 10
        if np.abs((np.pi / 2 - np.abs(self.sim.data.qpos[0] + self.sim.data.qpos[1] + self.sim.data.qpos[2]))) < 0.02:
            reward_angle = 1

        # REWARD FORCE
        force, torque = self.getForces(self.model.site_name2id('fingertip'), 'fingertip')
        Forces = np.array([force[0], force[1], force[2], torque[0], torque[1], torque[2]]).reshape(-1, 1)

        Forces_norm = np.linalg.norm([Forces[0], Forces[1]])
        if fingertip[0]< 0.03 and fingertip[0]>-0.03:
            reward_impedance=0
        else:
            if (Forces_norm > 2000):
                 reward_impedance = -0.1
            else:
                 reward_impedance = -Forces_norm / 20000
        angle = np.abs((np.pi / 2 - np.abs(self.sim.data.qpos[0] + self.sim.data.qpos[1] + self.sim.data.qpos[2])))
        if (np.linalg.norm(vec)<0.02 and angle<0.02 and  self.sim.data.time>0.1):
            final_reward = 20
            #final_reward= final_reward + (0.02-np.linalg.norm(vec))*1500 +  (0.02-angle)*1500
        else:
            final_reward = 0
        reward = reward_dist + reward_ctrl + reward_angle + reward_impedance + final_reward
        # data storage:
        if self.sim.data.time > 0:
            fingertip = self.get_body_com("fingertip")
            current_location = np.array([fingertip[0], fingertip[1],
                                         (self.sim.data.qpos[0] + self.sim.data.qpos[1] + self.sim.data.qpos[2])])
            self.current_location_tot.append(current_location)
            if (self.sim.data.time > (self.spec.max_episode_steps * self.dt - 2 * self.dt)):
                self.total_current_location = np.array(self.current_location_tot)
                # self.total_forces = np.array(self.forces_tot).transpose(0,2,1).reshape(-1,3)
                savetxt('current_location_tot.csv', self.total_current_location, delimiter=',')
        self.do_simulation(a, self.frame_skip)
        ob = self._get_obs()
        done = False



        return ob, reward, done, dict(reward_dist=reward_dist, reward_ctrl=reward_ctrl, reward_angle=reward_angle,
                                      reward_impedance=reward_impedance, final_reward=final_reward)

    def viewer_setup(self):
        self.viewer.cam.trackbodyid = 0

    def reset_model(self):
        while True:
            qpos = self.np_random.uniform(low=0, high=np.pi, size=self.model.nq) + self.init_qpos
            if ((qpos[1] + qpos[2]) < (1.23) * np.pi and (qpos[1] + qpos[2]) > -(1.23) * np.pi and qpos[1] < np.pi and
                    qpos[1] > -np.pi and qpos[2] < np.pi and qpos[2] > -np.pi):  # Avoiding singular states
                break
        self.current_location_tot = []
        self.xm = np.array([self.get_body_com("fingertip")[0], self.get_body_com("fingertip")[1],
                            (self.sim.data.qpos[0] + self.sim.data.qpos[1] + self.sim.data.qpos[2])]).reshape(-1, 1)
        self.xmDot = np.zeros(3).reshape(-1, 1)
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
            # self.goal = self.np_random.uniform(low=-.2, high=.2, size=2)
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
            self.sim.data.qpos.flat[3:5],
            self.sim.data.qvel.flat[:3],
            self.get_body_com("fingertip")[0:2] - self.get_body_com("target")[0:2],
            np.array(self.sim.data.qpos[0] + self.sim.data.qpos[1] + self.sim.data.qpos[2]).flat[0:]
        ])

    def getForces(self, site_id, bodyname):
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
            # force=-np.array(force)
            # torque=-np.array(torque)
            torque = [torque[0], torque[1], 0]
        if self.sim.data.ncon == 0:
            force = [0, 0, 0]
            torque = [0, 0, 0]
        return -np.array(force), -np.array(torque)

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
        xm0 = [xm[0, 0], xm[1, 0], xm[2, 0], xmDot[0, 0], xmDot[1, 0], xmDot[2, 0]]
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

    def build_x0(self, xr, vec):
        if (xr[0] >= 0 and xr[1] >= 0):
            x0 = np.linspace(xr[0], self.goal[0])
