import enum
import numpy as np
import scipy as sp
from SimWrapper.Controller import Controller
from SimWrapper.Enums import ControllerType, ImpedanceType
from SimWrapper.PID import PID

from scipy.integrate import odeint, ode, solve_ivp

class ImpedanceControl(PID):

    last_force = [0,0,0]
    last_v = [0,0,0]

    def __init__(self,ur5,type,k_imp, c_imp, m_imp,k_t_imp,c_t_imp,m_t_imp):
        PID.__init__(self,ur5)
        self.type = type.value
        self.k = k_imp
        self.c = c_imp
        self.m = m_imp
        self.k_t = k_t_imp
        self.c_t = c_t_imp
        self.m_t = m_t_imp
        self.gripper = 'gripper_finger'
        self.control_type = ControllerType.Impedance
        self.ic_integral = []
        self.pos_imp = np.array([0,0,0],np.dtype(np.float64))
        self.vel_imp = np.array([0,0,0],np.dtype(np.float64))
        self.theta_imp = np.array([0,0,0],np.dtype(np.float64))
        self.theta_dot = np.array([0,0,0],np.dtype(np.float64))
        self.dt = 2/1000;

        # self.ode_full_imp = ode(self.full_imp).set_initial_value(0,[0,0,0,0,0,0,0,0,0,0,0,0]).set_integrator('dopri5').set_solout(self.ode_callback)
        self.L = 0.1 #Gripper rod length
        self.r = [0,0,-self.L] #For calculating torque
        self.J = ((self.L ** 2) * self.m_t / 3)

    def get_imp(self,pos,orn,time=0, vimp = [0,0,0], aimp = [0,0,0]):
        if self.type == ImpedanceType.IMIC.value:
            self.imic_logic(pos,orn,time, vimp,aimp)
        elif self.type == ImpedanceType.IMPB.value:
            self.impb_logic(pos,orn)
        elif self.type == ImpedanceType.IMDB.value:
            self.imdb_logic(pos,orn)
        return np.dot(1,self.pos_imp)


    def impb_logic(self,pos,orn): #Not Implemented
        f = self.helper.get_sensor_force()
        # torque = force[3:6]
        # force = force[0:3]
        # torque = np.cross(self.r, force)
        x = self.pos_imp
        v =  self.vel_imp
        t0 = 0
        tspan = [t0, t0 + self.dt]
        # f = np.reshape([force, torque], 6)
        state = [x[0], x[1], x[2], v[0], v[1], v[2], self.theta_imp[0],self.theta_imp[1], self.theta_imp[2], self.theta_dot[0], self.theta_dot[1], self.theta_dot[2]]
        # sol = solve_ivp(self.full_imp, t, state,FT = 1)

        sol = solve_ivp(self.full_imp, tspan, state, args=(f,))
        # sol = solve_ivp(lambda t,y: self.full_imp(t,y,f), t_span=tspan, y0=state)

        # sol = 1
        index = sol.t.size - 1
        y = sol.y[:, index]
        self.pos_imp = y[0:3]
        self.vel_imp = y[3:6]
        self.theta_imp = y[6:9]
        # if (self.theta_imp[0] > 0):
        #     self.theta_imp[0] = np.minimum(0.25, self.theta_imp[0])
        # elif (self.theta_imp[0] < 0):
        #     self.theta_imp[0] = np.maximum(-0.05, self.theta_imp[0])
        self.theta_dot = y[9:12]

    def imic_logic(self,pos,orn,time, v0 = [0,0,0], a0 = [0,0,0]):
        x0 = np.dot(pos, 1/1000)
        x_im = self.helper.get_gripper_xpos()
        x = x_im-x0
        # x = -(self.helper.get_gripper_xpos()-np.dot(pos, 1/1000))
        v0 = np.dot(v0,1/1000)
        v_im = self.helper.get_gripper_vel()
        v = -(v_im-v0)
        # v = -(self.helper.get_gripper_vel()-np.dot(vimp,1/1000))
        # v = [0,0,0]
        orn_temp = self.helper.get_gripper_orn()
        tmp = np.dot(orn_temp, [0, 0, 1])
        x_theta = -np.pi / 2 - np.arctan2(orn_temp[2,2],orn_temp[2,1])
        x_theta = 0
        force = self.helper.get_sensor_force()
        torque = np.cross(self.r, force)
        force = np.add(force,np.dot(0*self.m/1000,a0))
        t0 = 0
        tspan= [t0, t0+self.dt]
        f = np.reshape([force,torque],6)
        state = [x[0],x[1],x[2],v[0],v[1],v[2],x_theta,0,0,0,0,0]
        # sol = solve_ivp(self.full_imp, t, state, args=(f,))
        # sol = solve_ivp(self.full_imp, t, state, args=(f,))
        sol = solve_ivp(lambda t,y: self.full_imp(t,y,f), t_span=tspan, y0=state)
        # sol = 1
        index = sol.t.size-1
        y = sol.y[:,index]
        self.pos_imp = y[0:3]
        # self.pos_imp[1] = self.pos_imp[1]
        # tmp_imp = self.pos_imp
        # self.pos_imp[0] = 0
        # self.pos_imp[1] = 0
        self.vel_imp = y[3:6]
        self.theta_imp = y[6:9]
        theta_dot = y[9:12]
        # self.ode_full_imp.set_f_params(force,torque)
        # self.ode_full_imp.integrate(self.dt)
        #
        # ximp = odeint(self.sim_model,x0, t, args=(Fx,))
        # yimp = odeint(self.sim_model,y0, t, args=(Fy,))
        # zimp = odeint(self.sim_model,z0, t, args=(Fz,))
        # self.pos_imp = [ximp[1,0],yimp[1,0], zimp[1,0]]
        # self.vel_imp = [ximp[1,1], yimp[1,1], zimp[1,1]]
        self.last_force = force


    def imdb_logic(self,pos,orn): #Not Implemented
        Jt = self.helper.get_JT(self.gripper);
        H = self.helper.get_H()
        tau = 0
        return 0

    #ODE model for impedance
    def sim_model(self,x0i, t0i, Fint):
        dxdt =  [x0i[1],(-self.c*x0i[1]-self.k*x0i[0]+Fint)/self.m]
        return dxdt

    def force_imp(self,t,y,Fint):
        return [y[1],(-self.c*y[1]-self.k*y[0]+Fint)/self.m]

    def torque_imp(self,t,y,Tint):
        return [y[1],(-self.c_t*y[1]-self.k_t*y[0]+Tint)/self.m_t]

    def full_imp(self, t, y, FT):
        return [y[3], y[4], y[5],
                (-self.c*y[3]-self.k*y[0]+FT[0])/self.m,
                (-self.c*y[4]-self.k*y[1]+FT[1])/self.m,
                (-self.c*y[5]-self.k*y[2]+FT[2])/self.m,
                y[9],y[10],y[11],
                (-self.c_t*y[9]-self.k_t*y[6]+FT[3])/self.J,
                (-self.c_t*y[10]-self.k_t*y[7]+FT[4])/self.J,
                (-self.c_t*y[11]-self.k_t*y[8]+FT[5])/self.J]
    # def full_imp(self, t, y, FT):
    #     return [y[3], y[4], y[5],
    #             -(-self.c*y[3]-self.k*y[0]+FT[0])/self.m,
    #             -(-self.c*y[4]-self.k*y[1]+FT[1])/self.m,
    #             -(-self.c*y[5]-self.k*y[2]+FT[2])/self.m,
    #             y[9],y[10],y[11],
    #             -(-self.c_t*y[9]-self.k_t*y[6]+FT[3])/self.J,
    #             -(-self.c_t*y[10]-self.k_t*y[7]+FT[4])/self.J,
    #             -(-self.c_t*y[11]-self.k_t*y[8]+FT[5])/self.J]
    def ode_callback(self,t,y):
        self.pos_imp = [y[0],y[1],y[2]]
        self.vel_imp = [y[3],y[4],y[5]]
        self.theta_imp = [y[6],y[7],y[8]]

    def reset_impedance(self):
        self.pos_imp = np.array([0,0,0],np.dtype(np.float64))
        self.vel_imp = np.array([0,0,0],np.dtype(np.float64))
        self.theta_imp = np.array([0,0,0],np.dtype(np.float64))
        self.theta_dot = np.array([0,0,0],np.dtype(np.float64))
        print("impedance reset")