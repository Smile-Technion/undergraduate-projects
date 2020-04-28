import math

import mujoco_py as mj
import numpy as np
from Sim.SimWrapper.Enums import ControllerType
from SimWrapper.Simulation import Simulator
from SimWrapper.InverseKinematics import IK
from SimWrapper.RoboticsHelper import RHELPER
from SimWrapper.Controller import Controller
from Sim.SimWrapper.PID import PID
from Sim.SimWrapper.DataHandler import DataObject

class Task:

    def __init__(self, simobj,controller):
        self.sim = simobj.sim
        self.simobj = simobj
        self.ik = IK()
        self.helper = RHELPER(self.simobj)
        self.dt = self.sim.model.opt.timestep
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

    def openGripper(self): #Not Implemented
        raise NotImplementedError
        #self.gripperOpen = True
        # maintain_pos = True  # for simobj.step()
        # Gripper_Gain = -100
        # for i in range(500):
        #     self.gripper_sync()
        #     self.sim.data.ctrl[6:8] = np.dot(Gripper_Gain, [1, 1])
        #     self.gripper_sync()
        #     self.simobj.step(maintain_pos)
        #     self.gripper_sync()
        #
        # if self.debug:
        #     print("gripper opened ")

    def closeGripper(self): #Not Implemented
        raise NotImplementedError
        #Gripper_Gain = 20
        # maintain_pos = True  # for simobj.step()
        # self.gripperOpen = False
        # # for i in range(500):
        # #     self.gripper_sync()
        # #     self.sim.data.ctrl[6:8] = np.dot(Gripper_Gain, [1, 1])
        # #     self.gripper_sync()
        # #     self.simobj.step(maintain_pos)
        # #     self.gripper_sync()
        # closed_pos = [1.12810781, -0.59798289, -0.53003607]
        # for i in range(2):
        #     for j in range(3):
        #         self.sim.data.qpos[6 + i * 4 + j] = closed_pos[j]

    def goTo(self, pos, orn, wait=1000, pos_target = [], noImpedance = False):
        wait = wait / 1000  # convert from ms to s
        start_time = self.sim.get_state().time
        pos = list(pos)
        orn = list(orn)
        e_per = np.dot(self.e_threshold, np.ones(6))
        e = np.ones(6)
        # e_integral = np.zeros(6)
        q_current = self.helper.get_current_joints()
        maintain_pos = False  # for simobj.step()
        q_des_p = self.ik.calcPos(q_current, pos, orn)
        if self.controller.control_type == ControllerType.Regular or (self.controller.control_type == ControllerType.Impedance and noImpedance):
            q_des = self.ik.calcPos(q_current, pos, orn)
            while ((np.abs(e) > e_per[0:6]).any()):
                e = q_des - self.helper.get_current_joints()
                u = self.controller.get_u(q_des)
                self.log_data_reg(pos,q_des,q_current,u,[0,0,0])
                self.sim.data.ctrl[:6] = np.dot(1 / 101, self.sim.data.qfrc_bias[0:6]) + u
                self.simobj.step(maintain_pos)
                if (self.sim.get_state().time - start_time) > wait:
                    if self.debug:
                        print("waypoint not reached")
                        return
                    break
            if self.debug:
                print("waypoint succed")
        elif self.controller.control_type == ControllerType.Impedance:
            pos_imp = [0,0,0]
            counter = 1
            while ((np.abs(e) > e_per[0:6]).any()):
                if (np.abs(self.helper.get_sensor_force()) > 0.01).any():
                    if len(pos_target) > 0:
                        pos_imp = self.controller.get_imp(pos_target, orn,wait)
                        pos_imp = pos_target+pos_imp-pos
                    else:
                        pos_imp = self.controller.get_imp(pos, orn,wait)
                    q_des = self.ik.calcPos(q_current, (np.add(pos, pos_imp)), orn)
                else:
                    q_des = q_des_p
                q_current = self.helper.get_current_joints()
                print(counter)
                counter += 1
                u = self.controller.get_u(q_des)
                e = q_des - q_current
                self.log_data_reg(pos, q_des, q_current, u, pos_imp)
                self.sim.data.ctrl[:6] = np.dot(1 / 101, self.sim.data.qfrc_bias[0:6]) + u
                self.simobj.step(maintain_pos)
                if (self.sim.get_state().time - start_time) > wait:
                    if self.debug:
                        print("waypoint not reached")
                        return
                    break
            if self.debug:
                print("waypoint succed")

    def verticalMovement(self, start_pos, target_pos,step_size = 5, step_wait = 50, start_wait = 2000,end_wait = 200):
        z_start = start_pos[2]
        z_end = target_pos[2]
        if (np.abs(z_start) < 2 ) and (np.abs(z_end) < 2):
            z_start *= 1000
            z_end   *= 1000
            s_pos = np.add(np.dot(start_pos,1000),[0,0,self.hoffset])
        else:
            s_pos = np.add(start_pos, [0,0,self.hoffset])
        orn = [np.pi, 0, np.pi / 2]
        increment = (z_end>z_start)*step_size - (z_start > z_end)*step_size
        noImpedance = False
        if increment > 0:
            noImpedance = True
            print("debug")
        self.goTo(s_pos,orn,start_wait)
        steps = np.arange(z_start+self.hoffset,z_end+self.hoffset, increment)
        s_pos = [s_pos[0],s_pos[1],0]
        pos_target = [s_pos[0],s_pos[1],z_end+self.hoffset]
        counter = 1
        for step in steps:
            step_pos = np.add(s_pos, [0,0,+step])
            if counter == len(steps):
                step_wait = end_wait
            self.goTo(step_pos,orn,step_wait,pos_target, noImpedance )
            counter += 1


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