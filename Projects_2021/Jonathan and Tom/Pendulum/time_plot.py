from numpy import sin, cos
from scipy.fft import fft, fftfreq
import numpy as np
import scipy.integrate as integrate
import matplotlib.pyplot as plt

# Pendulum Set up
class pendulum:
   def __init__(self,l,m,c,g):
       self.l = l
       self.m = m
       self.c = c
       self.g = g

# l: initial length of pendulum 1 in m
# m: mass of pendulum 1 in kg
# c: Damping of the joint
# Environmental Constant: acceleration due to gravity, in m/s^2

pen1 = pendulum(1,1,0,9.8)

def derivs(state, t):

    dthdt = np.zeros_like(state)

    dthdt[0] = - pen1.g/pen1.l * np.sin(state[1])  - pen1.c/pen1.m * state[0]

    dthdt[1] = state[0]

    return dthdt

#time array from 0..100 sampled at 0.05 second steps
dt = 0.05
t = np.arange(0, 20, dt)
# initial conditions
# th is initial angle,  w is initial angular velocity
w0 = 0
th0 = 20

# initial value for state vectors
state = [np.radians(w0),np.radians(th0)]

# integrate ODE to obtain the angle values
th = integrate.odeint(derivs, state, t)
omega_sol, theta_sol = th[:, 0], th[:, 1]
x = pen1.l*sin(theta_sol)
y = -pen1.l*cos(theta_sol)

# Do FFT analysis of array
FFT = fft(theta_sol)

# Getting the related frequencies
freqs = fftfreq(len(theta_sol), dt)     ## added dt, so x-axis is in meaningful units

# Create subplot windows and show plot
fig, axs = plt.subplots(2, 1)
plot1, = axs[0].plot(t, theta_sol)
axs[0].set_title(r'$\theta$ over time where $\theta_0 = $ %5.3f' %(th0))
axs[0].set_xlabel('Time')
axs[0].set_ylabel('Amplitude')
plot2, = axs[1].plot(freqs, np.log10(abs(FFT)), '.', label = 'fft')
plot3 = axs[1].axvline(1/(2*np.pi*np.sqrt(pen1.l/pen1.g)), 0.1, 0.9, color = 'k', label = r'$\frac{1}{2\pi}\sqrt{g/L}$')
plot4 = axs[1].axvline(1/(-2*np.pi*np.sqrt(pen1.l/pen1.g)), 0.1, 0.9, color = 'k')
axs[1].legend()
axs[1].set_title('FFT analysis')
axs[1].set_xlabel('Frequencies [Hz]')
axs[1].set_ylabel('Amplitude (log10)')
plt.tight_layout()
plt.show()



'''
# time plot, x and y
fig, axs = plt.subplots(1, 2)
plot1, = axs[0].plot(t, x)
axs[0].set_title('x over time')
axs[0].set_xlabel('time [s] ')
axs[0].set_ylabel('x [m]')
plot2, = axs[1].plot(t, y)
axs[1].set_title('y over time')
axs[1].set_xlabel('time [s] ')
axs[1].set_ylabel('y [m]')
plt.tight_layout()
plt.show()

# time plot, theta and omega
fig, axs = plt.subplots(1, 2)
plot1, = axs[0].plot(t, th[:,1])
axs[0].set_title(r'$\theta$ over time')
axs[0].set_xlabel(r'time [s] ')
axs[0].set_ylabel(r'$\theta$ [rad]')
plot2, = axs[1].plot(t, th[:,0])
axs[1].set_title(r'$\omega$ over time')
axs[1].set_xlabel(r'time [s] ')
axs[1].set_ylabel(r'$\omega$ [rad/s]')
plt.show()
'''