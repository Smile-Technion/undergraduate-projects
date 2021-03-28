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
        if np.linalg.norm(vec) >= 0.04:
            reward_dist = - np.linalg.norm(vec)
        if np.linalg.norm(vec)  < 0.04:
            if self.sim.data.time==0:
                reward_dist=0
            else:
                reward_dist = 0.20 - 3*np.linalg.norm(vec)

        anglevec=(np.pi / 2 - self.sim.data.qpos[0] - self.sim.data.qpos[1] - self.sim.data.qpos[2])
        if np.abs(anglevec) >= 0.03:
            reward_angle = - np.abs(anglevec)/10
        if np.abs(anglevec) < 0.03 and self.sim.data.time>0.02:
            reward_angle = 0.1 - 3 * np.abs(anglevec)/10


        # IMPEDANCE REWARD
        force, torque = self.getForces(self.model.site_name2id('fingertip'), 'fingertip')
        Forces = np.array([force[0], force[1], force[2], torque[0], torque[1], torque[2]]).reshape(-1, 1)
        ForcesSave=np.array([Forces[0],Forces[1],Forces[5]])

        #IMPEDANCE REWARD
        angle = (self.sim.data.qpos[0] + self.sim.data.qpos[1] + self.sim.data.qpos[2])
        x0 = np.array([fingertip[0], fingertip[1], angle]).reshape(-1, 1)
        x0_vel = np.array([self.sim.data.sensordata[0], self.sim.data.sensordata[1],
                           self.sim.data.qvel[0] + self.sim.data.qvel[1] + self.sim.data.qvel[2]]).reshape(-1, 1)
        ForcesODE=np.array([ForcesSave[0],ForcesSave[1],ForcesSave[2]]).reshape(1,-1)

        Forces_norm = np.linalg.norm(ForcesSave)

        reward_impedance = 0
        reward_impedanceangle = 0
        if Forces_norm == 0 or self.sim.data.time==0 :
            self.xm=x0
            self.xmdot=x0_vel

        if Forces_norm>0.00001 and fingertip[1]>0.2:
            [self.xm, self.xmdot, self.xmdotaim] = self.Impedance_ode(self.xm, self.xmDot, self.K, self.B, self.M,  x0, x0_vel, ForcesODE)
            # reward_dist=0
            # reward_angle=0
            vec_imp = fingertip[0:2] - np.array(self.xm[0:2]).reshape(1,-1)
            vec_imp_angle=anglevec-self.xm[2]
            reward_impedance = - np.linalg.norm(vec_imp)
            reward_impedanceangle=-np.double(np.abs(vec_imp_angle)/10)

        ##############
        #ORIGINAL FORCE REWARD:
        # if fingertip[0]< 0.03 and fingertip[0]>-0.03:
        #     reward_impedance=0
        # else:
        #     if (Forces_norm > 2000):
        #          reward_impedance = -0.1
        #     else:
        #          reward_impedance = -Forces_norm / 20000
        #############

        # FINAL REWARD:
        reward_angle=0
        reward_impedanceangle=0

        reward_final=0

        #yes angle case:

        # if np.linalg.norm(vec)<0.02 and np.abs(anglevec)<0.001 and self.sim.data.time>0.02:
        #     reward_final=0.2+(0.1-np.linalg.norm(vec)*5)**2 -np.abs(anglevec)*100

        #no angle case:

        if np.linalg.norm(vec)<0.02  and self.sim.data.time>0.02:
            reward_final=0.1+(0.1-np.linalg.norm(vec)*5)**2

        #REWARD SUMMING
        reward = reward_dist  + reward_angle + reward_impedance + reward_impedanceangle +  reward_final
        # data storage:
        if self.sim.data.time > 0:
            fingertip = self.get_body_com("fingertip")
            current_location = np.array([fingertip[0], fingertip[1],
                                         (self.sim.data.qpos[0] + self.sim.data.qpos[1] + self.sim.data.qpos[2])])
            xm=self.xm.reshape(1,-1)
            self.current_location_tot.append(current_location)
            self.forces.append(ForcesSave)
            self.xm_tot.append(xm)

            if (self.sim.data.time > (self.spec.max_episode_steps * self.dt - 2 * self.dt)):  # Final Step
                self.total_current_location = np.array(self.current_location_tot)
                self.total_forces = np.array(self.forces).transpose(0, 2, 1).reshape(-1, 3)
                self.total_xm=np.array(self.xm_tot).transpose(0, 2, 1).reshape(-1, 3)
                try:
                     savetxt('current_location_tot.csv', self.total_current_location, delimiter=',')
                     savetxt('forces.csv', self.total_forces, delimiter=',')
                     savetxt('xms.csv', self.total_xm, delimiter=',')
                except:
                        print("faild to save files")
        self.do_simulation(a, self.frame_skip)
        ob = self._get_obs()
        done = False



        return ob, reward, done, dict(reward_dist=reward_dist,
                                      reward_angle=reward_angle,
                                      reward_impedance=reward_impedance,
                                      reward_impedanceangle=reward_impedanceangle,
                                      reward_final=reward_final)

    def viewer_setup(self):
        self.viewer.cam.trackbodyid = 0

    def reset_model(self):
        while True:
            qpos = self.np_random.uniform(low=0, high=np.pi, size=self.model.nq) + self.init_qpos
            qpos=np.array([0.3,2,-0.5,0,0,0,0])
            if ((qpos[1] + qpos[2]) < (1.23) * np.pi and (qpos[1] + qpos[2]) > -(1.23) * np.pi and qpos[1] < np.pi and
                    qpos[1] > -np.pi and qpos[2] < np.pi and qpos[2] > -np.pi):  # Avoiding singular states
                break
        self.xm_tot = []
        self.forces = []
        self.current_location_tot = []
        self.xm = np.array([self.get_body_com("fingertip")[0], self.get_body_com("fingertip")[1],
                            (self.sim.data.qpos[0] + self.sim.data.qpos[1] + self.sim.data.qpos[2])]).reshape(-1, 1)
        self.xmDot = np.zeros(3).reshape(-1, 1)

        self.k = 1000  *1
        k = self.k
        self.m = 150 *1
        m = self.m
        self.b = 350 *1
        b = self.b


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
            self.goal = np.array([0.0, 0.293])
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
        force, torque = self.getForces(self.model.site_name2id('fingertip'), 'fingertip')
        Forces = np.array([force[0], force[1], force[2], torque[0], torque[1], torque[2]]).reshape(-1, 1)
        theta = self.sim.data.qpos.flat[:3]
        return np.concatenate([
            np.cos(theta),
            np.sin(theta),
            self.sim.data.qpos.flat[3:5],
            self.sim.data.qvel.flat[:3],
            self.get_body_com("fingertip")[0:2] - self.get_body_com("target")[0:2],
            np.array([Forces[0],Forces[1],Forces[5]]).flat[0:],
            np.array([self.xm[0],self.xm[1]]).reshape(1,-1).flat[0:],
            np.array([self.xmdot[0],self.xmdot[1]]).reshape(1,-1).flat[0:]
            # np.cos(np.array(self.sim.data.qpos[0] + self.sim.data.qpos[1] + self.sim.data.qpos[2]).flat[0:]),
            # np.sin(np.array(self.sim.data.qpos[0] + self.sim.data.qpos[1] + self.sim.data.qpos[2]).flat[0:]),
            # np.cos(self.xm[2]),
            # np.sin(self.xm[2]),
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
            torque = [-self.sim.data.sensordata[9], -self.sim.data.sensordata[10], -self.sim.data.sensordata[11]]
            torque = [torque[0], torque[1], torque[2]]
        if self.sim.data.ncon == 0:
            force = [0, 0, 0]
            torque = [0, 0, 0]
        return np.array(force), np.array(torque)

    def impedance_differential_equation(self, y, t, K, B, M, x0, x0dot, Fint):
        x1, y1, theta1, x2, y2, theta2 = y
        xm = [x1, y1, theta1]
        xm = np.array(xm).reshape(-1, 1)
        xmdot = [x2, y2, theta2]
        xmdot = np.array(xmdot).reshape(-1, 1)
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
        xmdotaim2 = (np.matmul((np.matmul(K, np.linalg.inv(M))), x0)).reshape(-1, 1) + \
                    (np.matmul((np.matmul(B, np.linalg.inv(M))), x0dot)).reshape(-1, 1) - \
                    (np.matmul(Fint, np.linalg.inv(M))).reshape(-1, 1) - \
                    np.matmul((np.matmul(K, np.linalg.inv(M))), xm2) - \
                    np.matmul((np.matmul(B, np.linalg.inv(M))), xmdot2)
        return xm2, xmdot2, xmdotaim2

    def build_x0(self, xr, vec):
        if (xr[0] >= 0 and xr[1] >= 0):
            x0 = np.linspace(xr[0], self.goal[0])
