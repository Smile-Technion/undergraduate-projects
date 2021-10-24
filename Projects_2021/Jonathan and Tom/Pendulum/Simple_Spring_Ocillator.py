from numpy import sin, cos
import numpy as np
import scipy.integrate as integrate
from scipy.fft import fft, fftfreq
import matplotlib.pyplot as plt



# Pendulum Set up
class spring:
   def __init__(self, l, m, k):
       self.l = l
       self.m = m
       self.k = k

# l: initial length of pendulum 1 in m
# m: mass of pendulum 1 in kg
# c: Damping of the joint
# k: Spring constant of the pendulum rod

spring1 = spring(2,1,50)

# Environmental Constant
g = 0  # acceleration due to gravity, in m/s^2

def derivs(state, t):

    dxdt = np.zeros_like(state)

    dxdt[0] = -spring1.k/spring1.m*(state[1]-spring1.l)

    dxdt[1] = state[0]


    return dxdt

#time array from 0..100 sampled at 0.05 second steps
dt = 0.01
t = np.arange(0, 20, dt)

# initial conditions
x0 = 5
v0 = 0

# initial value for state vectors
state = [v0, x0]

# integrate ODE to obtain the angle values
sol = integrate.odeint(derivs, state, t)
x_sol, v_sol = sol[:,1], sol[:,0]

# time plot, x and y
fig, axs = plt.subplots(2, 1)
plot1, = axs[0].plot(t, x_sol)
axs[0].set_title('x over time')
axs[0].set_xlabel('time [s] ')
axs[0].set_ylabel('x [m]')
plot2, = axs[1].plot(t, v_sol)
axs[1].set_title('v over time')
axs[1].set_xlabel('time [s] ')
axs[1].set_ylabel('v [m/s]')
plt.tight_layout()
plt.show()

# Do FFT analysis of array
FFT = fft(x_sol)

# Getting the related frequencies
freqs = fftfreq(len(x_sol), dt)     ## added dt, so x-axis is in meaningful units

# Create subplot windows and show plot
fig, axs = plt.subplots(2, 1)
plot1, = axs[0].plot(t, x_sol)
axs[0].set_title(r'x over time where $x_0 = $ %5.3f' %(x0))
axs[0].set_xlabel('Time')
axs[0].set_ylabel('Amplitude')
plot2, = axs[1].plot(freqs, np.log10(abs(FFT)), '.', label = 'fft')
plot3 = axs[1].axvline(1/(2*np.pi*np.sqrt(spring1.m/spring1.k)), 0.1, 0.9, color = 'k', label = r'$\frac{1}{2\pi}\sqrt{k/m}$')
plot4 = axs[1].axvline(1/(-2*np.pi*np.sqrt(spring1.m/spring1.k)), 0.1, 0.9, color = 'k')
axs[1].legend()
axs[1].set_title('FFT analysis')
axs[1].set_xlabel('Frequencies [Hz]')
axs[1].set_ylabel('Amplitude (log10)')
plt.tight_layout()
plt.show()

