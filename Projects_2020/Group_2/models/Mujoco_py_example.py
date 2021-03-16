import numpy as np
import mujoco_py as mj
from mujoco_py_renderer import   MujocoPyRenderer
from mujoco_py import (MjSim, load_model_from_xml,functions,
                       load_model_from_path, MjSimState,
                       ignore_mujoco_warnings,
                       load_model_from_mjb)


from matplotlib import pyplot as plt
import time

# model1 = load_model_from_path("bread-visual.xml")

model = load_model_from_path("UR5gripper.xml")
sim = MjSim(model)
viewer = MujocoPyRenderer(sim)



sim.reset()
    # After reset jacobians are all zeros
sim.forward()
target_jacp = np.zeros(3*sim.model.nv)
print("what is the shape")
print(np.shape(target_jacp))
target_jacr= np.zeros(3*sim.model.nv)



F=np.array([0,0,-9.81*1e-2,0,0,0]).T

#np.testing.assert_allclose(target_jacp, np.zeros(3 * sim.model.nv))
    # After first forward, jacobians are real
#sim.forward()
K_diag=2000
C_diag=100
A_diag=1e-3

K=np.identity(3)*K_diag
C=np.identity(3)*C_diag
A=np.identity(3)*A_diag

#K_diag=0.3
#C_diag=0.05


for i in range(3):
    K[i, i]=K_diag
    C[i,i]=C_diag
    A[i, i] = A_diag

phis=np.arange(0,100000*np.pi*5,0.01)
r=1
x = r*np.cos(phis)
y = r*np.sin(phis)
x_intial=sim.data.site_xpos[1]
print(x_intial)
x_desired=np.array([5,5,5])

v_intial=sim.data.site_xvelp[1]
v_desired=np.array([0,0,0])

a_desired=np.array([0,0,0])
a_intial=np.array([0,0,0])

dt=sim.model.opt.timestep
#sim.data.get_site_jacp('target', jacp=target_jacp)
    # Should be unchanged after steps (zero action)
graph=[]
for i in range(100000):
    x_desired=np.array([x[i],y[i],1])
    F[:3]=np.dot(K,x_desired-x_intial)+np.dot(C,v_desired-v_intial)+np.dot(A,a_desired-a_intial)
    H = np.zeros(sim.model.nv* sim.model.nv)
    functions.mj_fullM(sim.model, H,sim.data.qM)
    sim.data.get_site_jacp('gripperpalm', jacp=target_jacp)
    sim.data.get_site_jacr('gripperpalm', jacr=target_jacr)

    J_L = target_jacp.reshape((3,sim.model.nv))
    print("J_l")
    J_A = target_jacr.reshape((3,sim.model.nv))
    J = np.concatenate((J_L, J_A),axis=0)

    print(np.shape(J))
    H_L =np.dot(np.linalg.pinv(J_L.T),np.dot(H.reshape(sim.model.nv, sim.model.nv), np.linalg.pinv(J_L)))
    H_all=np.dot(np.linalg.pinv(J.T),np.dot(H.reshape(sim.model.nv, sim.model.nv), np.linalg.pinv(J)))



    # F_a=np.dot(A,0.3-sim.data.qacc)
    action = np.dot(J.T, np.dot(H_all, F[:6]))+sim.data.qfrc_bias
    # action = sim.data.qfrc_bias+np.dot(H.reshape(6,6),np.dot(J_L.T,F[:6]))
    print("action")
    print(type(action))
    # action =  np.dot(J.T, F)

    print(np.shape(action))
    #
    sim.data.ctrl[:] = action[:10]

    sim.step()
    sim.forward()
    #print(np.max(action))
    #print(sim.data.qacc)
    viewer.render()
    x_intial = sim.data.site_xpos[1]
    a_intial=(v_intial-sim.data.site_xvelp[1])/dt
    print(a_intial)
    v_intial = sim.data.site_xvelp[1]
    normal=np.linalg.norm(x_intial-x_desired)
    #print(normal)
    # if normal<0.1:
    #     print("in")
    #     if x_desired[0]==0:
    #         x_desired = np.array([-1, 0, 0.5])
    #     elif x_desired[0]==1:
    #         x_desired = np.array([0, 1, 0.3])
    #     elif x_desired[0] == -1:
    #         x_desired = np.array([1, 0, 0.5])


    graph.append(np.abs(x_intial-x[i]))
 #   sim.forward()

print("the desired is {} and the intial is{}".format(x_desired,x_intial))
plt.plot(graph)
plt.show()
