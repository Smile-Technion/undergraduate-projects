import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def inverse_kinematics(l1,l2,r,shift,n):
    phis=np.arange(0,2*np.pi*n,0.01)
    x = shift+r*np.cos(phis)
    y = shift+r*np.sin(phis)
    xx=np.multiply(x,x)
    yy=np.multiply(y,y)
    l1a = l1*l1*np.ones(len(x))
    l2a = l2*l2*np.ones(len(x))
    addxy=np.add(xx,yy)
    addl1l2=np.add(-l1a,-l2a)
    num = np.add(addxy,addl1l2)
    den = (2*l1*l2*np.ones(len(x)))
    check = np.divide(num, den)
    check = np.max(np.abs(check))
    # if check>1:
    #     print('Invalid configuration')
    #     break
    theta2=np.arccos(np.divide(num,den))
    theta1=np.arctan2(y,x)-np.arcsin(np.divide(l2*np.sin(theta2),np.sqrt(addxy)))

    return [theta1,theta2]
    # print(theta2)




l1=0.6
l2=0.4
r=0.3
n=2
shift = 0.4
[t1,t2]= inverse_kinematics(l1,l2,r,shift,n)
# plt.plot(np.arange(0, len(theta1)), theta2)
# plt.grid(True, 'major', 'both')
# plt.show()



fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-l1-l2, shift+r+l2), ylim=(-l1-l2, shift+r+l2))
ax.grid()

x1 = l1*np.cos(t1)
y1 = l1*np.sin(t1)

x2 = l2*np.cos(np.add(t2,t1)) + x1
y2 = l2*np.sin(np.add(t2,t1)) + y1
line, = ax.plot([], [], 'o-', lw=2)
time_template = 'time = %.1fs'
time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)
dt=1/30

def init():
    line.set_data([], [])
    time_text.set_text('')
    return line, time_text


def animate(i):
    thisx = [0, x1[i], x2[i]]
    thisy = [0, y1[i], y2[i]]

    line.set_data(thisx, thisy)
    time_text.set_text(time_template % (i*dt))
    return line, time_text

ani = animation.FuncAnimation(fig, animate, np.arange(1, len(t1)),
                              interval=1, blit=True, init_func=init)

ani.save('circle_trajectory_2link.mp4', fps=30)
plt.show()