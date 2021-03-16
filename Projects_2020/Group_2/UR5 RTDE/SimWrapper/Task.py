import math

import numpy as np
from SimWrapper.Enums import ControllerType
from SimWrapper.Simulation import Simulator
from SimWrapper.InverseKinematics import IK
from SimWrapper.RoboticsHelper import RHELPER
from SimWrapper.Controller import Controller
from SimWrapper.PID import PID
from SimWrapper.DataHandler import DataObject
from SimWrapper.PathCalc import path_calc_3_axis, path_calc_1_axis


class Task:

    def __init__(self, simobj,controller):
        self.sim = simobj.sim
        self.simobj = simobj
        self.ur5 = simobj
        self.ik = IK()
        self.helper = RHELPER(self.ur5)
        self.dt = 2/1000 #self.sim.model.opt.timestep
        self.e_threshold = 0.001
        self.gripperOpen = True
        self.pos = []
        self.orn = []
        self.joints = []
        self.debug = False
        self.controller = controller
        self.hoffset = -0.855*1000
        #Data Logging
        self.desired_vel = []
        self.desired_vel = []
        self.q_desired = []
        self.q_current = []
        self.control_effort = []
        self.desired_pos = DataObject()
        self.desired_pos_imp = DataObject()
        self.gripper_pos = DataObject()
        self.gripper_force = DataObject()
        self.gripper_vel = DataObject()
        self.last_orn  = [np.pi,0,0]
        self.last_pos = [0,0,0]

    def openGripper(self): #Not Implemented
        raise NotImplementedError
        #self.gripperOpen = True


    def closeGripper(self): #Not Implemented
        raise NotImplementedError

    def goTo(self, pos,orn, vel,acc,step_time = 2):
        pos = list(pos)
        orn = list(orn)
        pos_current = self.helper.get_gripper_xpos()
        q_current = self.helper.get_current_joints()
        pos_imp = self.controller.get_imp(pos, orn, step_time, vimp=vel, aimp=acc)
        print(pos_imp)
        orn_imp = np.add(orn, [-self.controller.theta_imp[0], -self.controller.theta_imp[1], -self.controller.theta_imp[2]])
         # q_des = self.ik.calcPos(q_current,pos_current,orn,(np.add(pos, pos_imp)),orn_imp)
        # pos_imp = [0,0,0]
        # print('\n','desired pos:',pos)
        # q_des = self.ik.calcPos(q_current, pos_current, orn,pos,orn)
        # print('\n','desired q:',q_des)
        q_des = [0,0,0,0,0,0]
        q_des[0] = pos[0]+pos_imp[0]
        q_des[1] = pos[1]+pos_imp[1]
        q_des[2] = pos[2]+pos_imp[2]
        # q_des[3] = orn[0]+orn_imp[0]
        # q_des[4] = orn[1]+orn_imp[1]
        # q_des[5] = orn[2]+orn_imp[2]

        q_des[3] = orn_imp[0]
        q_des[4] = orn_imp[1]
        q_des[5] = orn_imp[2]

        # # print(q_des,'q desired',sep='\n')
        # q_des =  np.dot((np.pi / 180), [-60, -108, -60, -94, 87, 152])
        self.ur5.step(q_des)

    #
    # def goTo(self, pos, orn,vel = [0,0,0], acc= [0,0,0], pos_target = [], noImpedance = False,isVert= False):
    #     pos = list(pos)
    #     orn = list(orn)
    #     e_per = np.dot(self.e_threshold, np.ones(6))
    #     e = np.ones(6)
    #     # e_integral = np.zeros(6)
    #     q_current = self.helper.get_current_joints()
    #     maintain_pos = False  # for simobj.step()
    #     q_des_p = self.ik.calcPos(q_current, pos, orn)
    #     if self.controller.control_type == ControllerType.Regular or (self.controller.control_type == ControllerType.Impedance and noImpedance):
    #         # if (self.controller.control_type == ControllerType.Impedance and noImpedance):
    #         #     orn = self.last_orn
    #         #     self.last_pos = np.dot(1000, self.helper.get_gripper_xpos())
    #         #     pos = [self.last_pos[0],self.last_pos[1], pos[2]]
    #         q_des = self.ik.calcPos(q_current, pos, orn)
    #         # self.controller.reset_impedance();
    #         while ((np.abs(e) > e_per[0:6]).any()):
    #             e = q_des - self.helper.get_current_joints()
    #             u = self.controller.get_u(q_des)
    #             self.log_data_reg(pos,q_des,q_current,u,[0,0,0])
    #             self.sim.data.ctrl[:6] = np.dot(1 / 101, self.sim.data.qfrc_bias[0:6]) + u
    #             self.simobj.step(maintain_pos)
    #         if self.debug:
    #             print("waypoint succed")
    #     elif self.controller.control_type == ControllerType.Impedance:
    #         pos_imp = [0,0,0]
    #         while ((np.abs(e) > e_per[0:6]).any()):
    #             # if (np.abs(self.helper.get_sensor_force()) > 0.01).any():
    #             #     if len(pos_target) > 0:
    #             #         pos_imp = self.controller.get_imp(pos_target, orn,wait)
    #             #         pos_imp = pos_target+pos_imp-pos
    #             #     else:
    #             #         pos_imp = self.controller.get_imp(pos, orn,wait)
    #             #     q_des = self.ik.calcPos(q_current, (np.add(pos, pos_imp)), np.add(orn,[0*self.controller.theta_imp[0],0,0]))
    #             #     self.last_orn = self.impedanceLinearPath(np.add(pos, pos_imp),orn,np.add(orn,[20*self.controller.theta_imp[0],0,0]),time=1000,step_time=50)
    #             #
    #             #     q_current = self.helper.get_current_joints()
    #             #     e = q_des - q_current
    #             # else:
    #             if (np.abs(self.helper.get_sensor_force()) > 0.01).any():
    #                 check = 1
    #             pos_imp = self.controller.get_imp(pos, orn, wait,vimp = vel, aimp = acc)
    #             print(self.controller.theta_imp[0])
    #             q_des = self.ik.calcPos(q_current, (np.add(pos, pos_imp)), np.add(orn,[1*self.controller.theta_imp[0],0,0]))
    #             q_current = self.helper.get_current_joints()
    #             u = self.controller.get_u(q_des)
    #             # q_current = self.helper.get_current_joints()
    #             e = q_des - q_current
    #             self.log_data_reg(pos, q_des, q_current, u, pos_imp)
    #             self.sim.data.ctrl[:6] = np.dot(1 / 101, self.sim.data.qfrc_bias[0:6]) + u
    #             self.simobj.step(maintain_pos)
    #             if (self.sim.get_state().time - start_time) > wait:
    #                 if self.debug:
    #                     print("waypoint not reached")
    #                     return
    #                 break
    #         if self.debug:
    #             print("waypoint succed")
    #
    # def impedanceLinearPath(self,pos_target,orn,orn_target, time=50, step_time=10):
    #     orn_sections = int(np.ceil((np.abs(orn[0]-orn_target[0])*180/np.pi)/0.1))
    #     x_0 = np.dot(1000,self.helper.get_gripper_xpos())
    #     v_0 = 0
    #     x_T = pos_target
    #     v_T = 0
    #     sections = int(np.ceil(np.max(np.abs(np.abs(x_0 - x_T) / 0.1))))
    #     if sections > 1:
    #         sec = np.maximum(orn_sections,sections)
    #         path = path_calc_3_axis(x_0,v_0,x_T,v_T,step_time)
    #         orns = orn[0]
    #         length = len(path[0])
    #         orns = np.linspace(orn,orn_target, sec)
    #         for i in range(length):
    #             pos = [path[0][i], path[1][i], path[2][i]]
    #             self.goTo(pos, orns[i], wait=step_time, noImpedance=True)
    #             if i * step_time > time:
    #                 return orns[i]
    #     else:
    #         orns = np.linspace(orn,orn_target,orn_sections)
    #         for i in range(orn_sections):
    #             self.goTo(pos_target, orns[i], wait=step_time, noImpedance=True)
    #             if i * step_time > time:
    #                 return orns[i]
    #     return orns[i]

    def goToByPath(self,x_T,orn,T_end = 2000, step_time = 2,ignoreImpedance = False):
        x_0 = self.helper.get_gripper_xpos()
        self.controller.reset_impedance()
        traj = path_calc_3_axis(x_0, 0, x_T, 0,T_end,step_time)
        for i in range(len(traj[0])):
            pos = [traj[0][i],traj[1][i],traj[2][i]]
            velocity = [traj[3][i],traj[4][i],traj[5][i]]
            acceleration = [traj[6][i],traj[7][i],traj[8][i]]
            self.goTo(pos,orn,vel = velocity, acc = acceleration)
    #     def goTo(self, pos,orn, vel,acc,step_time = 2):
    # def verticalMovement(self, start_pos, target_pos,step_size = 5, step_wait = 50, start_wait = 2000,end_wait = 200, ignoreImpedance=False):
    #     z_start = start_pos[2]
    #     z_end = target_pos[2]
    #     if (np.abs(z_start) < 2 ) and (np.abs(z_end) < 2):
    #         z_start *= 1000
    #         z_end   *= 1000
    #         s_pos = np.add(np.dot(start_pos,1000),[0,0,self.hoffset])
    #         target_pos = np.add(np.dot(target_pos,1000),[0,0,self.hoffset])
    #     else:
    #         s_pos = np.add(start_pos, [0,0,self.hoffset])
    #         target_pos = np.add(target_pos, [0,0,self.hoffset])
    #     orn = [np.pi, 0,0]
    #     increment = (z_end>z_start)*step_size - (z_start > z_end)*step_size
    #     noImpedance = False
    #     if increment > 0:
    #         noImpedance = True
    #         print("debug")
    #     self.goTo(s_pos,orn,start_wait)
    #     traj = path_calc_3_axis(s_pos,0,target_pos,0)
    #     for i in range(len(traj[0])):
    #         if i == len(traj[0]):
    #             step_wait = end_wait
    #         pos = [traj[0][i],traj[1][i],traj[2][i]]
    #         self.goTo(pos,orn,step_wait, noImpedance )


    # def verticalMovement(self, start_pos, target_pos,step_size = 5, step_wait = 50, start_wait = 2000,end_wait = 200):
    #     z_start = start_pos[2]
    #     z_end = target_pos[2]
    #     if (np.abs(z_start) < 2 ) and (np.abs(z_end) < 2):
    #         z_start *= 1000
    #         z_end   *= 1000
    #         s_pos = np.add(np.dot(start_pos,1000),[0,0,self.hoffset])
    #     else:
    #         s_pos = np.add(start_pos, [0,0,self.hoffset])
    #     orn = [np.pi, 0,0]
    #     increment = (z_end>z_start)*step_size - (z_start > z_end)*step_size
    #     noImpedance = False
    #     if increment > 0:
    #         noImpedance = True
    #         print("debug")
    #     self.goTo(s_pos,orn,start_wait)
    #     steps = np.arange(z_start+self.hoffset,z_end+self.hoffset, increment)
    #     s_pos = [s_pos[0],s_pos[1],0]
    #     pos_target = [s_pos[0],s_pos[1],z_end+self.hoffset]
    #     counter = 1
    #     for step in steps:
    #         step_pos = np.add(s_pos, [0,0,+step])
    #         if counter == len(steps):
    #             step_wait = end_wait
    #         self.goTo(step_pos,orn,step_wait,pos_target, noImpedance )
    #         counter += 1

    def slide(self, length=50,x = True, y= False, z = False, increments = 1, wait = 50,forward=True):
        current_xpos = np.dot(self.helper.get_gripper_xpos(False),1000)
        if forward:
            dir = 1
        else:
            dir = -1
        orn = [np.pi,0,np.pi/2]
        steps = np.linspace(0,length,int(np.round(length/increments)))
        for step in steps:
            step_pos = np.add(current_xpos, [step*x*dir, step*y*dir, step*z*dir+self.hoffset])
            self.goTo(step_pos,orn,wait)
    def moveBy(self,x=0,y=0,z=0,wait = 100):
        current_pos = np.dot(self.helper.get_gripper_xpos(),1000)
        orn = [np.pi, 0, np.pi / 2]
        new_pos = np.add(current_pos, [x,y,z])
        self.goTo(new_pos, orn, wait)


    def log_data_reg(self,desired_pos,q_desired,q_current,u,pos_imp):#taking positin and control effort from task
        self.q_desired.append(q_desired)
        self.q_current.append(q_current)
        self.control_effort.append(u)
        self.desired_pos.log_data(desired_pos)
        self.gripper_force.log_data(self.helper.get_sensor_force())
        self.gripper_pos.log_data(self.helper.get_gripper_xpos()*1000)
        self.gripper_vel.log_data(self.helper.get_gripper_vel())
        self.desired_pos_imp.log_data(np.add(desired_pos, pos_imp))