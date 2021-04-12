import numpy as np
from gym import utils
from mujoco_py import (functions)
from gym.envs.mujoco import mujoco_env
from scipy.integrate import odeint
from numpy import savetxt
from numpy import cos as cos
from numpy import sin as sin

class ProjectEnvImp1(mujoco_env.MujocoEnv, utils.EzPickle):
    def __init__(self):
        utils.EzPickle.__init__(self)
        mujoco_env.MujocoEnv.__init__(self, '2dof__force_sensing_env_mechanismHorizontal3.xml', 1)

    def step(self, a):
        if (self.sim.data.time == 0):
            reward_impedance = 0
            reward_panalty = 0
            reward = reward_impedance + reward_panalty
            self.xm = np.zeros(3).reshape(-1,1)
            self.xmDot = np.zeros(3).reshape(-1,1)
            self.x0_equation = np.zeros(3).reshape(-1,1)
            self.x0_vel_equation = np.zeros(3).reshape(-1,1)
            self.ForcesImp = np.zeros(3)
            self.do_simulation(a, self.frame_skip)
            ob = self._get_obs()
            done = False
            return ob, reward, done, dict(reward_impedance=reward_impedance, reward_panalty=reward_panalty)
        else:
            if(self.sim.data.time==self.dt):
                self.fingertip_start = self.get_body_com("fingertip")
                self.xm = np.array([self.fingertip_start[0], self.fingertip_start[1],(self.sim.data.qpos[0] + self.sim.data.qpos[1] + self.sim.data.qpos[2])]).reshape(-1, 1)
                self.xmDot = np.zeros(3).reshape(-1, 1)

                self.xPath = []
                self.yPath = []
                self.phiPath = []
                self.x_dot_Path = []
                self.y_dot_Path = []
                self.phi_dot_Path = []
                T = self.spec.max_episode_steps * self.dt
                self.t_list = np.linspace(0, T, num=np.int32(T / self.dt))
                kx = (self.goal[0] - self.fingertip_start[0]) / (T) ** 3
                kphi = (np.pi / 2 - (self.sim.data.qpos[0] + self.sim.data.qpos[1] + self.sim.data.qpos[2])) / T ** 3
                a5 = 6 / T ** 2
                a4 = -(15 / T)
                a3 = 10
                delta = 0.0001

                if self.goal[0] == self.fingertip_start[0]:
                    ky = (self.goal[1] - self.fingertip_start[1]) / T ** 3
                    for t in self.t_list:
                        self.x0 = self.goal[0]
                        self.y0 = ky * (a5 * (t ** 5) + a4 * (t ** 4) + a3 * (t ** 3)) + self.fingertip_start[1]
                        self.phi0 = kphi * (a5 * (t ** 5) + a4 * (t ** 4) + a3 * (t ** 3)) + (self.sim.data.qpos[0] + self.sim.data.qpos[1] + self.sim.data.qpos[2])

                        self.x0_vel = 0
                        self.y0_vel = ky * (a5 * 5 * (t ** 4) + a4 * 4 * (t ** 3) + 3 * a3 * (t ** 2))
                        self.phi0_vel = kphi * (a5 * 5 * (t ** 4) + a4 * 4 * (t ** 3) + 3 * a3 * (t ** 2))

                        self.xPath.append(self.x0)
                        self.yPath.append(self.y0)
                        self.phiPath.append(self.phi0)

                        self.x_dot_Path.append(self.x0_vel)
                        self.y_dot_Path.append(self.y0_vel)
                        self.phi_dot_Path.append(self.phi0_vel)
                    self.x0_vec = np.array([self.xPath, self.yPath, self.phiPath])
                    self.x0_vel_vec = np.array([self.x_dot_Path, self.y_dot_Path, self.phi_dot_Path])
                else:
                    for t in self.t_list:
                        ky = ((self.goal[1] - self.fingertip_start[1]) / (delta + self.goal[0] - self.fingertip_start[0]))
                        ky0 = (self.goal[0] * self.fingertip_start[1] - self.goal[1] * self.fingertip_start[0]) / (delta + self.goal[0] - self.fingertip_start[0])
                        self.x0 = kx * (a5 * (t ** 5) + a4 * (t ** 4) + a3 * (
                                    t ** 3)) + self.fingertip_start[0]
                        self.y0 = ky * self.x0 + ky0
                        self.phi0 = kphi * (a5 * (t ** 5) + a4 * (t ** 4) + a3 * (
                                    t ** 3)) + (
                                              self.sim.data.qpos[0] + self.sim.data.qpos[1] + self.sim.data.qpos[2])
                        self.x0_vel = kx * (a5 * 5 * (t ** 4) + a4 * 4 * (t ** 3) + 3 * a3 * (
                                    t ** 2))
                        self.y0_vel = ky * self.x0_vel
                        self.phi0_vel = kphi * (
                                    a5 * 5 * (t ** 4) + a4 * 4 * (t ** 3) + 3 * a3 * (
                                       t ** 2))

                        self.xPath.append(self.x0)
                        self.yPath.append(self.y0)
                        self.phiPath.append(self.phi0)
                        self.x_dot_Path.append(self.x0_vel)
                        self.y_dot_Path.append(self.y0_vel)
                        self.phi_dot_Path.append(self.phi0_vel)
                    self.x0_vec = np.array([self.xPath, self.yPath, self.phiPath])
                    self.x0_vel_vec = np.array([self.x_dot_Path, self.y_dot_Path, self.phi_dot_Path])
                # logging xm, xmdot, current location and current velocity
                self.xm1 = np.array(self.xm).reshape(1,-1)
                self.xm_tot.append(self.xm1)
                self.xmDot1 = np.array(self.xmDot).reshape(1,-1)
                self.xmDot_tot.append(self.xmDot1)
                current_location1 = np.array([self.fingertip_start[0],self.fingertip_start[1],(self.sim.data.qpos[0] + self.sim.data.qpos[1] + self.sim.data.qpos[2])])
                current_velocity1 = np.array([self.sim.data.sensordata[0],self.sim.data.sensordata[1],(self.sim.data.qvel[0]+self.sim.data.qvel[1]+self.sim.data.qvel[2])])
                self.current_location_tot.append(current_location1)
                self.current_velocity_tot.append(current_velocity1)
                self.x0_equation = self.x0_vec[:,np.int(self.sim.data.time/self.dt-1)]
                self.x0_vel_equation = self.x0_vel_vec[:,np.int(self.sim.data.time/self.dt-1)]

                reward_impedance = 0
                reward_panalty = 0
                reward = reward_impedance
                self.do_simulation(a, self.frame_skip)
                ob = self._get_obs()
                done = False
                return ob, reward, done, dict(reward_impedance=reward_impedance, reward_panalty=reward_panalty)
            else:
                fingertip = self.get_body_com("fingertip")
                current_location = np.array([fingertip[0],fingertip[1],(self.sim.data.qpos[0] + self.sim.data.qpos[1] + self.sim.data.qpos[2])]).reshape(-1,1)
                current_velocity = np.array([self.sim.data.sensordata[0],self.sim.data.sensordata[1],(self.sim.data.qvel[0]+self.sim.data.qvel[1]+self.sim.data.qvel[2])]).reshape(-1,1)
                current_location1 = np.array([fingertip[0],fingertip[1],(self.sim.data.qpos[0] + self.sim.data.qpos[1] + self.sim.data.qpos[2])])
                current_velocity1 = np.array([self.sim.data.sensordata[0],self.sim.data.sensordata[1],(self.sim.data.qvel[0]+self.sim.data.qvel[1]+self.sim.data.qvel[2])])
                self.current_location_tot.append(current_location1)
                self.current_velocity_tot.append(current_velocity1)
                #reward_impedance_spring = 0
                force, torque = self.getForces(self.model.site_name2id('fingertip'), 'fingertip')
                Forces = np.array([force[0], force[1], force[2], torque[0], torque[1], torque[2]]).reshape(-1, 1)
                self.ForcesImp = ((np.array([-Forces[0], -Forces[1], -Forces[5]]))).reshape(1, -1)
                self.forces_tot.append(self.ForcesImp)
                self.x0_equation = self.x0_vec[:,np.int(self.sim.data.time/self.dt-1)]
                self.x0_vel_equation = self.x0_vel_vec[:,np.int(self.sim.data.time/self.dt-1)]
                [self.xm, self.xmDot, self.xmDotaim] = self.Impedance_ode(self.xm, self.xmDot, self.K, self.B, self.M,
                                                                          self.x0_equation, self.x0_vel_equation, self.ForcesImp)
                self.xm1 = np.array(self.xm).reshape(1,-1)
                self.xm_tot.append(self.xm1)
                self.xmDot1 = np.array(self.xmDot).reshape(1,-1)
                self.xmDot_tot.append(self.xmDot1)
                if(self.sim.data.time>(self.spec.max_episode_steps * self.dt-2*self.dt)):
                    self.total_xm = np.array(self.xm_tot).transpose(0,2,1).reshape(-1,3)
                    self.total_xmDot = np.array(self.xmDot_tot).transpose(0,2,1).reshape(-1,3)
                    self.total_current_location = np.array(self.current_location_tot)
                    self.total_current_velocity = np.array(self.current_velocity_tot)
                    self.total_forces = np.array(self.forces_tot).transpose(0,2,1).reshape(-1,3)
                    savetxt('x0.csv', self.xPath, delimiter=',')
                    savetxt('y0.csv', self.yPath, delimiter=',')
                    savetxt('phi0.csv', self.phiPath, delimiter=',')
                    savetxt('x0_vel.csv', self.x_dot_Path, delimiter=',')
                    savetxt('y0_vel.csv', self.y_dot_Path, delimiter=',')
                    savetxt('phi0_vel.csv', self.phi_dot_Path, delimiter=',')
                    savetxt('xm_tot.csv', self.total_xm, delimiter=',')
                    savetxt('xmDot_tot.csv', self.total_xmDot, delimiter=',')
                    savetxt('current_location_tot.csv', self.total_current_location, delimiter=',')
                    savetxt('current_velocity_tot.csv', self.total_current_velocity, delimiter=',')
                    savetxt('forces_tot.csv', self.total_forces, delimiter=',')

                impedance_vec =  current_location - self.xm
                impedance_norm_x = np.linalg.norm(impedance_vec[0])
                impedance_norm_y = np.linalg.norm(impedance_vec[1])
                impedance_norm_phi = np.linalg.norm(impedance_vec[2])

                if(impedance_norm_x >= 0.01):
                    reward_impedance_xm_x = -0.5
                if(impedance_norm_y >= 0.01):
                    reward_impedance_xm_y = -0.5
                if(impedance_norm_phi >= 0.01):
                    reward_impedance_xm_phi = -0.5

                if ((impedance_norm_x < 0.01) and (impedance_norm_x > 0.001)):
                    reward_impedance_xm_x = - impedance_norm_x
                if ((impedance_norm_y < 0.01) and (impedance_norm_y > 0.001)):
                    reward_impedance_xm_y = - impedance_norm_y
                if ((impedance_norm_phi < 0.01) and (impedance_norm_phi > 0.001)):
                    reward_impedance_xm_phi = - impedance_norm_phi
                if (impedance_norm_x < 0.001):
                    reward_impedance_xm_x = + 20
                if (impedance_norm_y < 0.001):
                    reward_impedance_xm_y = + 30
                if (impedance_norm_phi < 0.001):
                    reward_impedance_xm_phi = + 10
                #if (impedance_norm_x < 0.001 and impedance_norm_y < 0.001 and impedance_norm_phi < 0.002 and current_location1[1] >= 0.2):
                #    reward_impedance_spring = +50
                #else:
                #    reward_impedance_spring = 0
                reward_impedance_xm = reward_impedance_xm_x + reward_impedance_xm_y + reward_impedance_xm_phi# + reward_impedance_spring# +reward_accuracy_x + reward_accuracy_y + reward_accuracy_phi

                impedance_velocity_vec = current_velocity - self.xmDot
                impedance_velocity_norm_x = np.linalg.norm(impedance_velocity_vec[0])
                impedance_velocity_norm_y = np.linalg.norm(impedance_velocity_vec[1])
                impedance_velocity_norm_phi = np.linalg.norm(impedance_velocity_vec[2])

                if (impedance_velocity_norm_x > 0.02):
                    reward_impedance_xmDot_x = -0.5
                if (impedance_velocity_norm_y > 0.02):
                    reward_impedance_xmDot_y = -0.5
                if (impedance_velocity_norm_phi > 0.02):
                    reward_impedance_xmDot_phi = -0.5

                if ((impedance_velocity_norm_x < 0.02) and (impedance_velocity_norm_x > 0.01)):
                    reward_impedance_xmDot_x = - impedance_velocity_norm_x
                if ((impedance_velocity_norm_y < 0.02) and (impedance_velocity_norm_y > 0.01)):
                    reward_impedance_xmDot_y = - impedance_velocity_norm_y
                if ((impedance_velocity_norm_phi < 0.02) and (impedance_velocity_norm_phi > 0.01)):
                    reward_impedance_xmDot_phi = - impedance_velocity_norm_phi

                if (impedance_velocity_norm_x < 0.01):
                    reward_impedance_xmDot_x = + 15
                if (impedance_velocity_norm_y < 0.01):
                    reward_impedance_xmDot_y = + 20
                if (impedance_velocity_norm_phi < 0.01):
                    reward_impedance_xmDot_phi = + 10
                reward_impedance_xmDot = reward_impedance_xmDot_x + reward_impedance_xmDot_y + reward_impedance_xmDot_phi
                reward_impedance = reward_impedance_xmDot + reward_impedance_xm
                reward_panalty = 0
                reward = reward_impedance + reward_panalty
                self.do_simulation(a, self.frame_skip)
                ob = self._get_obs()
                done = False
                return ob, reward, done, dict(reward_impedance=reward_impedance, reward_panalty=reward_panalty)
    def viewer_setup(self):
        self.viewer.cam.trackbodyid = 0

    def reset_model(self):
        #qpos = self.np_random.uniform(low=0, high=np.pi, size=self.model.nq) + self.init_qpos
        qpos = np.array([np.pi*5/4, -np.pi*3/4, -np.pi *2/ 6, 0, 0, 0, 0])
        #fingertip=self.get_body_com("fingertip")

        self.x0_tot = []
        self.x0_vel_tot = []
        self.xm_tot = []
        self.xmDot_tot = []
        self.forces_tot = []
        self.current_location_tot = []
        self.current_velocity_tot = []
        self.index = 0

        self.k = 80
        k = self.k
        self.b = 20
        b = self.b
        self.m = 15
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
            self.goal = np.array([0, 0.26])
            if np.linalg.norm(self.goal) < 0.4:
                break
        qpos[3:5] = self.goal
        qpos[5] = -0.008
        qpos[6] = 0.008
        #qvel = self.init_qvel + self.np_random.uniform(low=-.005, high=.005, size=self.model.nv)
        qvel = np.zeros(self.model.nv)
        qvel[3:5] = 0
        qvel[5:7] = 0
        self.set_state(qpos, qvel)
        return self._get_obs()

    def _get_obs(self):
        theta = self.sim.data.qpos.flat[:3]
        x0_temp = self.x0_equation.flat[0:]
        x0Dot_temp = self.x0_vel_equation.flat[0:]
        fingertip_vel = np.array([self.sim.data.sensordata[0], self.sim.data.sensordata[1],
                                     (self.sim.data.qvel[0] + self.sim.data.qvel[1] + self.sim.data.qvel[2])])
        return np.concatenate([
            np.cos(theta), #cosine of joints angles
            np.sin(theta), #sine of joints angles
            self.sim.data.qpos.flat[3:5], #location of the target
            self.sim.data.qvel.flat[:3], #velocities of the joints
            self.get_body_com("fingertip")[0:2] - self.get_body_com("target")[0:2], # location of the end effector with relation to the target
            np.cos(np.array(self.sim.data.qpos[0] + self.sim.data.qpos[1] + self.sim.data.qpos[2]).flat[0:]), # end effector angle
            np.sin(np.array(self.sim.data.qpos[0] + self.sim.data.qpos[1] + self.sim.data.qpos[2]).flat[0:]),
            self.get_body_com("fingertip")[0:2] - x0_temp[0:2], # impedance state
            np.cos(np.array(self.sim.data.qpos[0] + self.sim.data.qpos[1] + self.sim.data.qpos[2]).flat[0:] - np.array(x0_temp[2]).flat[0:]),
            np.sin(np.array(self.sim.data.qpos[0] + self.sim.data.qpos[1] + self.sim.data.qpos[2]).flat[0:] - np.array(x0_temp[2]).flat[0:]),
            fingertip_vel[0:2]- x0Dot_temp[0:2], # impedance velocity state
            np.array(fingertip_vel[2]).flat[0:] - np.array(x0Dot_temp[2]).flat[0:],
            self.ForcesImp.flat[0:3],
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
                if contact.geom1 == 8 or contact.geom2 == 8: #depends of how many links
                    oritemp=np.array(self.sim.data.contact[i].frame).reshape(3,3)
                    functions.mj_contactForce(self.sim.model,self.sim.data, i, ctemp)
                    ctemp[0:3]=np.dot(np.linalg.inv(oritemp),np.array(ctemp[0:3]))
                    ctemp[3:6] = np.dot(np.linalg.inv(oritemp),np.array(ctemp[3:6]))
                    csum+=ctemp
            force  = [-csum[0],-csum[1],-csum[2]]
            torque=[-self.sim.data.sensordata[9],-self.sim.data.sensordata[10],-self.sim.data.sensordata[11]]
        if self.sim.data.ncon==0:
            force=[0,0,0]
            torque=[0,0,0]
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
    def outputArrange(self,vec):
        sizeF = 0
        for i in range(len(vec)):
            for j in range(len(vec[i])):
                sizeF = sizeF + 1

        vec_width=len(vec[0][0])
        full_vec = np.zeros([sizeF, vec_width])
        sizeF = 0
        for i in range(len(vec)):
            for j in range(len(vec[i])):
                full_vec[sizeF, :] = np.array(vec[i][j])
                sizeF = sizeF + 1
        return full_vec