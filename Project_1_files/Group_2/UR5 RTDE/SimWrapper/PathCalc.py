
import numpy as np

def path_calc_3_axis(x_0, v_0, x_T, v_T,T_end, step_time = 2 ):
    t = range(0,T_end, step_time)
    polyx=trajectory_calc_1_axis(x_0[0],v_0, x_T[0], v_T,T_end)
    polyy=trajectory_calc_1_axis(x_0[1],v_0, x_T[1], v_T,T_end)
    polyz=trajectory_calc_1_axis(x_0[2],v_0,  x_T[2], v_T,T_end)
    x = np.polyval(polyx,t)
    y = np.polyval(polyy,t)
    z = np.polyval(polyz,t)
    vx = polydiff(polyx,t)
    vy = polydiff(polyy,t)
    vz = polydiff(polyz,t)
    ax = polydiff(polyx,t,order = 2)
    ay = polydiff(polyy,t,order = 2)
    az = polydiff(polyz,t,order = 2)
    return [x,y,z,vx,vy,vz,ax,ay,az]

# def path_calc_1_axis(x_0, v_0, x_T, v_T,T_end, step_time = 2 ):
#     t = range(0,T_end, step_time)
#     polyx=path_calc_1_axis(x_0[0],v_0, x_T[0], v_T,T_end)
#     x = polyx[0] + polyx[1]*t + polyx[2] * t**2 + polyx[3] * t**3 + polyx[4] * t**4 + polyx[5] * t ** 5
#     return x

def path_calc_1_axis(x_0, v_0, x_T, v_T,T_end, step_time = 2 ):
    t = range(0,T_end, step_time)
    polyx=trajectory_calc_1_axis(x_0,v_0, x_T, v_T,T_end)
    x = np.polyval(polyx,t)
    vx = polydiff(polyx,t)
    ax = polydiff(polyx,t,order = 2)
    return [x,vx,ax]


def polydiff(poly,t, order = 1):
    poly_diff = np.polyder(poly,order)
    diff_eval = np.polyval(poly_diff, t)
    return diff_eval

def trajectory_calc_1_axis(x_0, v_0, x_T, v_T,T_end):
    T = T_end
    n = (x_T-x_0)/np.linalg.norm(x_T-x_0)
    A = [[1,0,0,0,0,0],
         [1,T,T**2,T**3,T**4,T**5],
         [0,1,0,0,0,0],
         [0,1,2*T,3*(T**2),4*(T**3),5*(T**4)],
         [0,0,2,0,0,0],
         [0,0,2,6*T,12*(T**2),20*(T**3)]]
    B = [x_0,x_T,v_0,v_T,0,0]
    sol = np.flip(np.linalg.solve(A,B))
    return sol
