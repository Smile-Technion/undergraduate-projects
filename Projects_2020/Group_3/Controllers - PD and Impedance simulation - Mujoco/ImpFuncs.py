from matplotlib import pyplot as plt
from numpy import cos as cos
from numpy import sin as sin
from numpy import savetxt
import numpy as np
from mujoco_py import (functions)
from numpy import diff
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import GeneralFuncs as GF
import globals
L1=globals.L1
L2=globals.L2
L3=globals.L3
SimStep=globals.SimStep
waitingTime=globals.waitingTime
sim=globals.sim
viewer=globals.viewer

#General impedance navigation function:
def impNav2Target(qd, qdDot, qdDotaim, P, D,dofs,qnormal,qdnormal,forcebool):
    target_jacp = np.zeros(3 * sim.model.nv)
    target_jacr = np.zeros(3 * sim.model.nv)

    # output parameters:
    effector_pos = []
    sensorData = []
    control_effort = []
    angle = []
    angleVel = []
    #Preperation
    sim.data.get_site_jacp('target', jacp=target_jacp)
    sim.data.get_site_jacr('target', jacr=target_jacr)
    J_L = target_jacp.reshape((3, sim.model.nv))
    J_A = target_jacr.reshape((3, sim.model.nv))
    J = np.concatenate((J_L, J_A), axis=0)
    J=J[0:6,0:int(dofs[0])]

    # H creation:
    H = np.zeros(sim.model.nv * sim.model.nv)
    functions.mj_fullM(sim.model, H, sim.data.qM)
    H = H.reshape([sim.model.nv, sim.model.nv])
    H=H[0:int(dofs[0]),0:int(dofs[0])] #depends on the robot size

    J_transpose = np.transpose(J)
    h = sim.data.qfrc_bias[0:int(dofs[0])]
    h=h.reshape(-1,1)
    error_q = qd - np.array(sim.data.actuator_length).reshape(-1,1)
    error_q_dot = qdDot - np.array(sim.data.actuator_velocity).reshape(-1,1)
    Fint,Torque=getForces(3, 'gripper')
    Fint=np.array([Fint[0],Fint[1],Fint[2],Torque[0],Torque[1],Torque[2]]).reshape(-1,1)
    #Output torques:
    action = -np.matmul(J_transpose, Fint) + h + np.matmul(H, (np.array(qdDotaim).reshape(-1,1) + np.matmul(P, error_q) + np.matmul(D, error_q_dot)))

    ##This part can be applied if you want to activate Impedance control in specific conditions:
    # if np.linalg.norm(Fint)==0 and forcebool==0:
    #     error = np.array(qnormal).reshape(1,3) - sim.data.actuator_length
    #     q_dot = sim.data.actuator_velocity - np.array(qdnormal).reshape(1,3)
    #     action = np.matmul(P, error.reshape(-1,1)) - np.matmul(D, q_dot.reshape(-1,1)) +np.array([sim.data.qfrc_bias[0], sim.data.qfrc_bias[1],sim.data.qfrc_bias[2]]).reshape(-1,1)  # sim.data.qfrc_bias is Gravity bias/robot dependant

    # Force collection:
    forces, torques = getForces(3, 'gripper')

    # collecting sensors data: particular to 2Dof Robot
    sensorData.append([forces[0], forces[1], forces[2],torques[0],torques[1],torques[2]])
    effector_pos.append([np.array(sim.data.site_xpos[0])[0], np.array(sim.data.site_xpos[0])[1],
                             np.array(sim.data.site_xpos[0])[2]])
    control_effort.append([action[0,0], action[1,0],action[2,0]])
    angle.append([sim.data.actuator_length[0], sim.data.actuator_length[1]])
    angleVel.append([sim.data.actuator_velocity[0], sim.data.actuator_velocity[1]])

    return effector_pos, sensorData, control_effort, action, angle, angleVel  # collecting position of endeffector

# one point navigating function with impedance control.
# input: target in joints domain , velocity target in velocity joints domain.
# boolean that is 1 if it is the last point in the path
# t_range - length of loop in simulation steps. relevant in reaching steady state in the last point.
def straightLineNavimp(x_vec,xDot_vec, nPts,P,D,K,M,B,xm,xmDot,methodParams,dofs,elbow,q_vec, q_dot_vec,forcebool):
    #Data collection:
    jointDataTotal = []
    jointVelDataTotal = []
    effector_pos_total = []
    sensorDataTotal = []
    control_effort_total = []
    prevJac=getCurrentSimJac(dofs)
    xmDotaim_vec = []
    xm_total=[]
    for i in range(nPts):
        Fint, Torque = getForces(3, 'gripper')

        if np.linalg.norm(Fint)>0:
            forcebool=1
        #stepping curJac
        currJac = getCurrentSimJac(dofs)
        #from now jacs can be derivated because they have step difference
        Force, Torque = getForces(3, 'gripper')
        #force into RK4 with negative sign
        Force=[-Force[0],-Force[1],-Torque[2]] #gets into RK4 function- not output - minus is not understandable

        #Calling impedance ode solver:
        [xm, xmDot, xmDotaim] = Impedance_ode(xm, xmDot, K, B, M, x_vec[:, i], xDot_vec[:, i], Force[0:3])

        if methodParams == 'analytical':
            [qd,qdDot,qdDotaim] = GF.getDesiredParamsAnalytic(xm,xmDot,xmDotaim)
        if methodParams == 'numerical':
            JacDer=numerical_der(currJac,prevJac)
            [qd, qdDot, qdDotaim] = getDesiredParamsNumeric(xm, xmDot, xmDotaim,currJac,JacDer,elbow,dofs)
        effector_pos, sensorData, control_effort, action, angle, angleVel = impNav2Target([qd[0], qd[1],qd[2]], [qdDot[0], qdDot[1],qdDot[2]],[qdDotaim[0],qdDotaim[1],qdDotaim[2]],P,D,dofs,[q_vec[i, 0], q_vec[i, 1], q_vec[i, 2]], [q_dot_vec[i, 0], q_dot_vec[i, 1],q_dot_vec[i, 2]],forcebool)

        #Output appending:
        xm_total.append(xm)
        jointDataTotal.append(angle)
        jointVelDataTotal.append(angleVel)
        effector_pos_total.append(effector_pos)
        sensorDataTotal.append(sensorData)
        control_effort_total.append(control_effort)

        # Action and rendering:
        sim.data.ctrl[:] = np.array(action).reshape(1,-1)
        sim.step()
        sim.forward()
        viewer.render()
        # Jacobian stepping
        prevJac=currJac
    #Arrange vectors:
    xm_total = np.array(xm_total).reshape(nPts, 3)
    full_joints_data = GF.outputArrange(jointDataTotal)
    full_joints_vel_data = GF.outputArrange(jointVelDataTotal)
    full_sensor_data = GF.outputArrange(sensorDataTotal)
    full_effector_data = GF.outputArrange(effector_pos_total)
    full_control_data = GF.outputArrange(control_effort_total)

    savetxt('xmDotaim.csv',xmDotaim_vec,delimiter=',')
    savetxt('xms.csv',xm_total,delimiter=',')


    return full_effector_data, full_sensor_data, full_control_data ,full_joints_data , full_joints_vel_data,xm

#function that gets q_dot at real time using simulation jacobian.
#now its specific to 2 DOF robot. needed to be manipulated to be
#compatible to other robots.
def getCurrentSimJac(dofs):
    target_jacp = np.zeros(3 * sim.model.nv)
    target_jacr = np.zeros(3 * sim.model.nv)
    sim.data.get_site_jacp('target', jacp=target_jacp)
    sim.data.get_site_jacr('target', jacr=target_jacr)
    J_L = target_jacp.reshape((3, sim.model.nv))
    J_A = target_jacr.reshape((3, sim.model.nv))
    J = np.concatenate((J_L, J_A), axis=0)
    J=J[0:6,0:int(dofs[0])]
    return J

def get_q_dot_RealTime(x_vec_dot,dofs):
    global SimStep
    J=getCurrentSimJac(dofs)
    #Jfinal = J[0:2, 0:2] #  This line is specific for each robot
    x_vec_dot_final = np.array([np.float(x_vec_dot[0]),np.float(x_vec_dot[1]),0,0,0,np.float(x_vec_dot[2])]).reshape(-1,1)
    J_inv=np.linalg.pinv(J)
    qdot_vec = np.matmul(J_inv, x_vec_dot_final)
    return (np.transpose(qdot_vec))
# function that returns full Jacobian of current simStep - directly from the simulation

#End effector forces (only contact forces)
def getForces(site_id, bodyname):
    force=[0,0,0]
    torque=[0,0,0]
    oritem=np.zeros([3,3], dtype=np.float64)
    ctemp = np.zeros(6, dtype=np.float64)
    csum = np.zeros(6, dtype=np.float64)
    if sim.data.ncon>0: #number of contacts in the simulation
        for i in range(sim.data.ncon):
            contact = sim.data.contact[i]
            if contact.geom1 == 5 or contact.geom2 == 5: #depends of how many links
                oritemp=np.array(sim.data.contact[i].frame).reshape(3,3)
                functions.mj_contactForce(sim.model,sim.data, i, ctemp)
                ctemp[0:3]=np.dot(np.linalg.inv(oritemp),np.array(ctemp[0:3]))
                ctemp[3:6] = np.dot(np.linalg.inv(oritemp),np.array(ctemp[3:6]))
                csum+=ctemp
        force  = [-csum[0],-csum[1],-csum[2]]
        torque=[-sim.data.sensordata[9],-sim.data.sensordata[10],-sim.data.sensordata[11]]

        force=np.array(force)
        torque=np.array(torque)

    if sim.data.ncon==0:
        force=[0,0,0]
        torque=[0,0,0]
    return np.array(force), np.array(torque)

#Previous ode solver that we do not use anymore
def RK4(K,B,M,F,inX,inXdot,inXm,inXmdot):
    inX=np.array(inX).reshape(3,1) #depends on number of dofs
    inXdot= np.array(inXdot).reshape(3, 1)
    m0=np.dot(SimStep,inXmdot)
    k0=SimStep*((np.matmul((np.matmul(K, np.linalg.inv(M))), inX)).reshape(-1,1)    + \
                (np.matmul((np.matmul(B, np.linalg.inv(M))), inXdot)).reshape(-1,1) - \
                          (np.matmul(F,np.linalg.inv(M))).reshape(-1,1)          - \
                np.matmul((np.matmul(K, np.linalg.inv(M))), inXm)    - \
                np.matmul((np.matmul(B, np.linalg.inv(M))), inXmdot) )
    x1_1 = inXm + m0/2    #mid variable x1_1
    x2_1 = inXmdot + k0/2 #mid variable x2_1
    m1=np.dot(SimStep,x2_1) #mid variable m1

    k1 =SimStep*((np.matmul((np.matmul(K, np.linalg.inv(M))), inX)).reshape(-1,1)    + \
                 (np.matmul((np.matmul(B, np.linalg.inv(M))), inXdot)).reshape(-1,1) - \
                           (np.matmul(F,np.linalg.inv(M))).reshape(-1,1)          - \
                 np.matmul((np.matmul(K, np.linalg.inv(M))), x1_1)   - \
                 np.matmul((np.matmul(B, np.linalg.inv(M))), x2_1))
    x1_2= inXm    + m1/2 #mid variable x1_2
    x2_2= inXmdot + k1/2 #mid variable x2_2
    m2 = np.dot(SimStep,x2_2)#mid variable m2

    k2 =SimStep*((np.matmul((np.matmul(K, np.linalg.inv(M))), inX)).reshape(-1,1)    + \
                 (np.matmul((np.matmul(B, np.linalg.inv(M))), inXdot)).reshape(-1,1) - \
                           (np.matmul(F,np.linalg.inv(M))).reshape(-1,1)          - \
                 np.matmul((np.matmul(K, np.linalg.inv(M))), x1_2)   - \
                 np.matmul((np.matmul(B, np.linalg.inv(M))), x2_2) )

    x1_3= inXm + m2 #mid variable x1_3
    x2_3= inXmdot + k2 #mid variable x1_3
    m3 = np.dot(SimStep, x2_3)#mid variable m3


    k3 =SimStep*((np.matmul((np.matmul(K, np.linalg.inv(M))), inX)).reshape(-1,1)    + \
                 (np.matmul((np.matmul(B, np.linalg.inv(M))), inXdot)).reshape(-1,1) - \
                           (np.matmul(F,np.linalg.inv(M))).reshape(-1,1)          - \
                 np.matmul((np.matmul(K, np.linalg.inv(M))), x1_3)   - \
                 np.matmul((np.matmul(B, np.linalg.inv(M))), x2_3) )

    xmF  =  inXm + (1/6) * (m0+2*m1+2*m2+m3)
    vmF  =  inXmdot + (1/6) * (k0+2*k1+2*k2+k3)
    amF  = np.matmul((np.matmul(K, np.linalg.inv(M))), (inX-xmF)) + \
           np.matmul((np.matmul(B, np.linalg.inv(M))), (inXdot-vmF))-(np.matmul(F,np.linalg.inv(M))).reshape(-1,1)
    #xmF=setXmintoRegion(xmF)
    return xmF, vmF, amF

#Function that returs x into real working region of the robot. specific for 2 dof robot - basic case
def setXmintoRegion(x):
    outPut=x
    if x[0] > 1.8:
        outPut[0]=1.8
    if x[1] > 1.8:
        outPut[1] = 1.8
    return outPut

# function that numerically derivates 2 points of a vector.
def numerical_der(vec, vec_prev):
    global SimStep
    der = (np.array(vec) - np.array(vec_prev)) / SimStep
    return der


#get desired impedance params:
def getDesiredParamsNumeric(xm,xmDot,xmDotaim,Jac,JacDer,elbow,dofs):
    qd = GF.invkin(xm[0],xm[1],xm[2],elbow)
    qd = qd[0:int(dofs[0])]
    qdDot=get_q_dot_RealTime([xmDot[0], xmDot[1],xmDot[2]],dofs)
    qdDot=np.array(qdDot).reshape(-1,1)
    qdDotaim=get_q_dotaimNum(qdDot,xmDotaim,Jac,JacDer)
    return qd, qdDot, qdDotaim

def get_q_dotaimNum(qdDot,x_dotaim,J,Jd):
        J_inv=np.linalg.pinv(J)
        x_dotaim_vec = np.array([np.float(x_dotaim[0]),np.float(x_dotaim[1]),0,0,0,np.float(x_dotaim[2])]).reshape(-1,1)
        qdotaim_vec = np.matmul(J_inv,x_dotaim_vec-np.matmul(Jd,qdDot))

        return (np.array(qdotaim_vec))

##current ode solver function:

# x1 = x_m, x2 = x_m_dot, y1= y_m, y2 = y_m_dot, theta1 = theta_m, theta2 = theta_m_dot
def impedance_differential_equation(y, t, K, B, M, x0, x0dot,Fint):
    x1, y1, theta1, x2, y2, theta2 = y
    xm = [x1, y1, theta1]
    xm = np.array(xm).reshape(-1,1)
    xmdot = [x2, y2, theta2]
    xmdot = np.array(xmdot).reshape(-1,1)
    dydt = [xmdot, (np.matmul((np.matmul(K, np.linalg.inv(M))), x0)).reshape(-1,1)    + \
                (np.matmul((np.matmul(B, np.linalg.inv(M))), x0dot)).reshape(-1,1) - \
                          (np.matmul(Fint,np.linalg.inv(M))).reshape(-1,1)          - \
                np.matmul((np.matmul(K, np.linalg.inv(M))), xm)    - \
                np.matmul((np.matmul(B, np.linalg.inv(M))), xmdot)]
    dydt1 = np.array(dydt[0])
    dydt2 = np.array(dydt[1])
    dydt = np.append(dydt1, dydt2)
    return dydt


def Impedance_ode(xm, xmDot, K, B, M, x0, x0dot, Fint):
    xm0 = [xm[0, 0], xm[1, 0], xm[2, 0], xmDot[0, 0], xmDot[1, 0], xmDot[2, 0]]
    sol = odeint(impedance_differential_equation, xm0, [0, SimStep], args=(K, B, M, x0, x0dot, Fint))
    sol1 = [sol[1, 0], sol[1, 1], sol[1, 2], sol[1, 3], sol[1, 4], sol[1, 5]]
    sol1 = np.array(sol1)
    xm = [sol1[0], sol1[1], sol1[2]]
    xm2 = np.array(xm).reshape(-1,1)
    xmdot = [sol1[3], sol1[4], sol1[5]]
    xmdot2 = np.array(xmdot).reshape(-1,1)
    #Fint = 10 * np.array(Fint)
    xmdotaim2 = (np.matmul((np.matmul(K, np.linalg.inv(M))), x0)).reshape(-1,1)    + \
                (np.matmul((np.matmul(B, np.linalg.inv(M))), x0dot)).reshape(-1,1) - \
                          (np.matmul(Fint,np.linalg.inv(M))).reshape(-1,1)          - \
                np.matmul((np.matmul(K, np.linalg.inv(M))), xm2)    - \
                np.matmul((np.matmul(B, np.linalg.inv(M))), xmdot2)
    return xm2, xmdot2, xmdotaim2