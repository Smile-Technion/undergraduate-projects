#!/usr/bin/env python
# Copyright (c) 2016, Universal Robots A/S,
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Universal Robots A/S nor the names of its
#      contributors may be used to endorse or promote products derived
#      from this software without specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL UNIVERSAL ROBOTS A/S BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import sys

sys.path.append('..')
import logging
from time import sleep
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config

import time
import numpy as np
import SimWrapper.PathCalc

# logging.basicConfig(level=logging.INFO)
ROBOT_HOST = '192.168.1.103'
ROBOT_PORT = 30004
config_filename = 'rtde/control_loop_configuration.xml'

keep_running = True

logging.getLogger().setLevel(logging.INFO)

conf = rtde_config.ConfigFile(config_filename)
state_names, state_types = conf.get_recipe('state')
setp_names, setp_types = conf.get_recipe('setp')
# watchdog_names, watchdog_types = conf.get_recipe('watchdog')

con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
con.connect()

# get controller version
con.get_controller_version()

# setup recipes
con.send_output_setup(state_names, state_types)
setp = con.send_input_setup(setp_names, setp_types)
# watchdog = con.send_input_setup(watchdog_names, watchdog_types)
# Setpoints to move the robot to
# setp1 = [-1.5, -1.4, -1.4, -1.3, 1.6, -0.3]
setp1 = np.dot((np.pi / 180), [-108, -60, -94, 87, 152])
setp2 = np.dot((np.pi / 180), [-115, -108, -60, -94, 87, 152])
path = SimWrapper.PathCalc.path_calc_1_axis(-60 * np.pi / 180, 0, -100 * np.pi / 180, 0,1000, 2)
# print(path[0])#!/usr/bin/env python
# # Copyright (c) 2016, Universal Robots A/S,
# # All rights reserved.
# # Redistribution and use in source and binary forms, with or without
# # modification, are permitted provided that the following conditions are met:
# #    * Redistributions of source code must retain the above copyright
# #      notice, this list of conditions and the following disclaimer.
# #    * Redistributions in binary form must reproduce the above copyright
# #      notice, this list of conditions and the following disclaimer in the
# #      documentation and/or other materials provided with the distribution.
# #    * Neither the name of the Universal Robots A/S nor the names of its
# #      contributors may be used to endorse or promote products derived
# #      from this software without specific prior written permission.
# # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# # ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# # WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# # DISCLAIMED. IN NO EVENT SHALL UNIVERSAL ROBOTS A/S BE LIABLE FOR ANY
# # DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# # (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# # LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# # ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# # SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# import sys
# sys.path.append('..')
# import logging
# from time import sleep
# import rtde.rtde as rtde
# import rtde.rtde_config as rtde_config
#
# import time
# import numpy as np
# import examples.PathCalc
#
#
# #logging.basicConfig(level=logging.INFO)
# ROBOT_HOST = '192.168.1.103'
# ROBOT_PORT = 30004
# config_filename = 'control_loop_configuration.xml'
#
# keep_running = True
#
# logging.getLogger().setLevel(logging.INFO)
#
# conf = rtde_config.ConfigFile(config_filename)
# state_names, state_types = conf.get_recipe('state')
# setp_names, setp_types = conf.get_recipe('setp')
# watchdog_names, watchdog_types = conf.get_recipe('watchdog')
#
# con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
# con.connect()
#
# # get controller version
# con.get_controller_version()
#
# # setup recipes
# con.send_output_setup(state_names, state_types)
# setp = con.send_input_setup(setp_names, setp_types)
# # watchdog = con.send_input_setup(watchdog_names, watchdog_types)
# # Setpoints to move the robot to
# # setp1 = [-1.5, -1.4, -1.4, -1.3, 1.6, -0.3]
# setp1 =  np.dot((np.pi/180),[-108,-60, -94,87,152,1])
# setp2 = np.dot((np.pi/180),[-115,-108,-60, -94,87,152])
# path = examples.PathCalc.path_calc_1_axis(-60 * np.pi / 180, 0, -100 * np.pi / 180,0, 1000,2)
# # print(path[0])
# p_len = len(path[0])
# # print(p_len)
# setp.input_double_register_0 = 0
# setp.input_double_register_1 = 0
# setp.input_double_register_2 = 0
# setp.input_double_register_3 = 0
# setp.input_double_register_4 = 0
# setp.input_double_register_5 = 0
# setp.input_double_register_6 = 0
#
# # The function "rtde_set_watchdog" in the "rtde_control_loop.urp" creates a 1 Hz watchdog
# # watchdog.input_int_register_0 = 0
#
# def setp_to_list(setp):
#     list = []
#     for i in range(0,7):
#         list.append(setp.__dict__["input_double_register_%i" % i])
#     return list
#
# def list_to_setp(setp, list):
#     for i in range (0,7):
#         setp.__dict__["input_double_register_%i" % i] = list[i]
#     return setp
#
# #start data synchronization
# if not con.send_start():
#     sys.exit()
#
# j = 0
# dir = 0
# # control loop
# while keep_running:
#     # receive the current state
#     state = con.receive()
#
#     if state is None:
#         break;
#
#     # do something...
#
#     if state.output_int_register_0 != 0:
#         start_time = time.clock()
#         new_setp = np.append(path[0][j], setp1)
#         list_to_setp(setp, new_setp)
#         # send new setpoint
#         con.send(setp)
#         # print(path[0][j])
#         # print(j)
#         # sleep(1)
#         print('exec. time',state.actual_execution_time)
#         # print(state.actual_TCP_force)#aquire force (fx,fx,fz)
#         # print(state.actual_TCP_pose)#(x,y,z,rx,ry,rz)
#         # print('exacute time [msec] is',(time.clock() - start_time)*1000)
#         if (dir == 0):
#             j=j+1
#             if j > p_len-1:
#                 j = p_len-2
#                 dir = 1
#         else:
#             j=j-1
#             if j < 1:
#                 j = 0
#                 dir = 0
#
#     # kick watchdog
#     # con.send(watchdog)
# con.send_pause()
# con.disconnect()
p_len = len(path[0])
# print(p_len)
setp.input_double_register_0 = 0
setp.input_double_register_1 = 0
setp.input_double_register_2 = 0
setp.input_double_register_3 = 0
setp.input_double_register_4 = 0
setp.input_double_register_5 = 0



# The function "rtde_set_watchdog" in the "rtde_control_loop.urp" creates a 1 Hz watchdog
# watchdog.input_int_register_0 = 0

def setp_to_list(setp):
    list = []
    for i in range(0, 6):
        list.append(setp.__dict__["input_double_register_%i" % i])
    return list


def list_to_setp(setp, list):
    for i in range(0, 6):
        setp.__dict__["input_double_register_%i" % i] = list[i]
    return setp


# start data synchronization
if not con.send_start():
    sys.exit()

j = 0
dir = 0
# control loop
while keep_running:
    # receive the current state
    state = con.receive()

    if state is None:
        break;

    # do something...

    if state.output_int_register_0 != 0:
        start_time = time.clock()
        new_setp = np.append(path[0][j], setp1)
        list_to_setp(setp, new_setp)
        # send new setpoint
        con.send(setp)
        # print(path[0][j])
        # print(j)
        # sleep(1)
        print('exec. time', state.actual_execution_time)
        # print(state.actual_TCP_force)#aquire force (fx,fx,fz)
        # print(state.actual_TCP_pose)#(x,y,z,rx,ry,rz)
        # print('exacute time [msec] is',(time.clock() - start_time)*1000)
        if (dir == 0):
            j = j + 1
            if j > p_len - 1:
                j = p_len - 2
                dir = 1
        else:
            j = j - 1
            if j < 1:
                j = 0
                dir = 0

    # kick watchdog
    # con.send(watchdog)
con.send_pause()
con.disconnect()
