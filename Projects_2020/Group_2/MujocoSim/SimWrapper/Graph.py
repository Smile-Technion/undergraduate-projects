import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import mujoco_py as mj
from Sim.SimWrapper.DataHandler import DataObject
from Sim.SimWrapper.Enums import ControllerType

class Graph:
    import numpy as np

    import matplotlib
    import matplotlib.pyplot as plt
    def __init__(self,task,start_time = 0):
        self.task=task
        if task.controller.control_type == ControllerType.Impedance:
            prefix = 'Impedance_'
            self.isImpedance = True
        else:
            prefix = 'Regular_'
            self.isImpedance = False

        print(prefix)
        self.name_prefix = prefix
        self.start_time = start_time
        self.dt=self.task.simobj.model.opt.timestep
        self.contact_forces()
        self.position()

    def position(self):
        gripper_desired_pos = self.task.desired_pos
        gripper_pos = self.task.gripper_pos
        gripper_desired_imp_pos = self.task.desired_pos_imp
        if self.isImpedance:
            ex = (gripper_desired_imp_pos.x - gripper_pos.x)
            ey = (gripper_desired_imp_pos.y - gripper_pos.y)
            ez = (gripper_desired_imp_pos.z - gripper_pos.z)
            title = 'Position Error - Relative to Impedance'
            t = np.arange(0, len(ex) * self.dt, self.dt)
            self.plot3(t, ex, ey, ez, 'pos_error_imp', ttl=title, ylbl='Error [mm]')

        ex = (gripper_desired_pos.x - gripper_pos.x)
        ey = (gripper_desired_pos.y - gripper_pos.y)
        ez = (gripper_desired_pos.z - gripper_pos.z)
        title = 'Position Error - Relative to Desired Position'
        t = np.arange(0, len(ex) * self.dt, self.dt)
        self.plot3(t,ex,ey,ez, 'pos_error', ttl=title, ylbl='Error [mm]')


    def contact_forces(self):
        gripper_force = self.task.gripper_force
        fx = gripper_force.x
        fy = gripper_force.y
        fz = gripper_force.z
        t = np.arange(0, len(fx) * self.dt, self.dt)
        self.plot(t,fx,'contact_force_x', ttl='Contact Force - X Axis')
        self.plot(t,fy,'contact_force_y', ttl='Contact Force - Y Axis')
        self.plot(t,fz,'contact_force_z', ttl='Contact Force - Z Axis')


    def plot(self,x,y,name= [], ttl='', xlbl='Time [sec]', ylbl='Force [N]'):
        if len(name) == 0:
            name = np.random(1e3)
        fig, ax = plt.subplots()
        ax.plot(x,y)
        ax.grid()
        ax.set_xlim(self.start_time)
        ax.set_ylim(auto=True)
        ax.set(xlabel=xlbl, ylabel=ylbl)
        plt.title(ttl)
        fig.savefig(self.name_prefix+ name+'.png')

    def plot3(self,x,y1,y2,y3,name= [], ttl='', xlbl='Time [sec]', ylbl='Force [N]'):
        if len(name) == 0:
            name = np.random(1e3)
        fig, ax = plt.subplots()
        ax.plot(x,y1)
        ax.plot(x,y2)
        ax.plot(x,y3)
        plt.gca().legend(('X','Y','Z'))
        ax.grid()
        ax.set_xlim(self.start_time)
        ax.set_ylim(-50,50)
        ax.set(xlabel=xlbl, ylabel=ylbl)
        plt.title(ttl)
        fig.savefig(self.name_prefix+ name+'.png')