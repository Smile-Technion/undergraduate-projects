import numpy as np
import globals
pi=np.pi
L1=globals.L1
L2=globals.L2
L3=globals.L3
SimStep=globals.SimStep
waitingTime=globals.waitingTime

import GeneralFuncs as GF
import ImpFuncs as IF

#Choose controller paramers:
p=450 #18
d=20 #0.3

#Choose Impedance paramers (used only when with Impedance controller):
k=1.26
m=0.18
b=0.8

#Defining diagonal matrices:
#controller:
P = np.array([[p, 0, 0],
             [0, p, 0],
             [0 , 0, p]])
D = np.array([[d, 0, 0],
             [0, d, 0],
            [0 , 0 , d]])

#Impedance:
K = np.array([[k, 0 ,0],
                [0, k, 0],
                [0, 0, k]])
M = np.array([[m, 0, 0],
                [0, m, 0],
                [0, 0, m]])
B = np.array([[b, 0, 0],
                [0, b, 0],
                [0, 0, b]])

##Define Path for the endeefector to navigate (X,Y,angle@rad)
#Path can be built up from 2 points or more.
wantedPath = [[0.245,1.2,pi/2],[0.245,2,pi/2]]
#Times vector - should be in size(wantedPath)-1.
wantedTimes = [2]

#metodology has 3 inputs:
#1. 'numerical' . method to recive wanted path derivation (velocity vector)
#2. 'imp'/'PD' - method of control. imp-> PB-IC Impedance controller implementation according to Zacksenhouse and Valency (2003)
# PD - classic PD + gravity compensation controller
#3. 'numerical' . method to recive q,qd,qd2 for the impedance controller.
# in previous versions of the software for 2DOF robot: analytical methods were avaliable in sections 1 and 3 but we disabled those options.

#3. 'numerical'/'analytical' -
methodology=['numerical','imp','numerical']
while 2>1:
    scenario=globals.scenario
    GF.Navigate(K,M,B,P,D,wantedPath,wantedTimes,waitingTime,methodology[0],methodology[1],methodology[2])




