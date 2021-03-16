import math
import sys
sys.path.append('..')
import logging
from time import sleep
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import threading

from SimWrapper.Simulation import Simulator



import numpy as np
import time




class RTDECon(Simulator):
    maintain_pose = False
    initial_state = [np.pi/2, -np.pi/2, np.pi/2, 0, 0, 0] # [1.86977729,1.24937109,2.00859607,0.90643269, 1.54136571, 0.29779118]  #[np.pi/2, -np.pi/2, np.pi/2, 0, 0, 0]
    state1 = [ 1.72891478, -1.36432606,  2.03544137,  0.99842782,  1.55507632,  0.15733697]
    initial_state = state1
    #[1.86977729 - 1.24937109  2.00859607  0.90643269  1.54136571  0.29779118]
    def __init__(self, config_filename="rtde/control_loop_configuration.xml"):
        # exec(open('clear_registers.py').read())
        # logging.basicConfig(level=logging.INFO)
        self.sim = []
        ROBOT_HOST = '192.168.1.103'
        ROBOT_PORT = 30004
        keep_running = True
        conf = rtde_config.ConfigFile(config_filename)
        state_names, state_types = conf.get_recipe('state')
        setp_names, setp_types = conf.get_recipe('setp')

        self.con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
        self.con.connect()
        # get controller version
        self.con.get_controller_version()

        # setup recipes
        self.con.send_output_setup(state_names, state_types)
        self.setp = self.con.send_input_setup(setp_names, setp_types)
        self.con.send_start()
        self.set_initial_state()
        # self.update_thread = threading.Thread(target=self.update(), args=(1,))
        # self.update_thread.start()
        # self.update_thread.join()

    #Set robot joints to its initial state
    def set_initial_state(self):
        self.setp.input_double_register_0 = 0
        self.setp.input_double_register_1 = 0
        self.setp.input_double_register_2 = 0
        self.setp.input_double_register_3 = 0
        self.setp.input_double_register_4 = 0
        self.setp.input_double_register_5 = 0
        self.state = self.con.receive()
        self.con.send(self.setp)
        self.state = self.con.receive()
        # if not self.con.send_start():
        #     print('Initializting error')
        #     sys.exit()

    def setp_to_list(self, setp):
        list = []
        for i in range(0, 6):
            list.append(self.setp.__dict__["input_double_register_%i" % i])
        return list

    def list_to_setp(self,setp, list):
        # for i in range(0, 6):
        #     self.setp.__dict__["input_double_register_%i" % i] = list[i]

        # watchdog = con.send_input_setup(watchdog_names, watchdog_types)
        # Setpoints to move the robot to
        # setp1 = [-1.5, -1.4, -1.4, -1.3, 1.6, -0.3]
        setp.input_double_register_0 = list[0]
        setp.input_double_register_1 = list[1]
        setp.input_double_register_2 = list[2]
        setp.input_double_register_3 = list[3]
        setp.input_double_register_4 = list[4]
        setp.input_double_register_5 = list[5]
        # setp.input_double_register_6 = list[6]
        return setp

    def update(self):
        while 1:
            self.state = self.con.receive()
            sleep(1/1000)

    def step(self, new_setp):
        self.state = self.con.receive()
        if self.state.output_int_register_0 != 0:
            # new_setp = [-1.05, -1.85, -1.05,-1.64,1.51, 2.65]
            self.setp = self.list_to_setp(self.setp, new_setp)
            # send new setpoint
            # print('desired joints angle', self.setp.input_double_register_0)
            self.con.send(self.setp)


