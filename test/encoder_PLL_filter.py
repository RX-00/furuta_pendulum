import numpy as np
import matplotlib.pyplot as plt
import math

# root-mean square
def rms(e):
    return np.sqrt(np.mean(e*e))

# max absolute value
def maxabs(e):
    return max(np.abs(e))


# Our Measured and Exact Position Samples
t = np.linspace(0, 4, 5000)
pos_exact = (np.abs(t-0.5) - np.abs(t-1.5) - np.abs(t-2.5) + np.abs(t-3.5))/2
pos_measured = pos_exact + 0.04*np.random.randn(t.size)

fig = plt.figure(figsize=(8, 6),dpi=80)
ax = fig.add_subplot(1, 1, 1)
ax.plot(t,pos_measured,'.',markersize=1.5)
ax.plot(t,pos_exact,'k')
ax.set_ylim(-0.5, 1.5)
ax.set_title('Exact and Measured Position')
ax.set_ylabel('position')
ax.set_xlabel('time')

(-0.5, 1.5)


# A simple example of a PLL is where the phase detector is a multiplier
# that acts on sine waves, the loop filter is an integrator, and there
# is no feedback filter (just passthrough)
#
# The phase detector outputs a sum and difference frequency:
# sum:  if the output frequency is the about same, then the sum term
#       cos(phi_i(t) + phi_o(t)) is about the input frequency
# diff: cos(phi_i(t) - phi_o(t)) is at a low frequency
#
# The loop filter is designed to filter out the double-frequency term,
# and integrate the low-frequency term
# This will reach equilibrium with constant phase difference between
# phi_i(t) and phi_o(t) -> the loop locks onto the input phase!
#t = np.linspace(0, 5, 100000)
def simpll(tlist, A, B, omega0, phi0):
    def helper():
        phi = phi0
        x = -omega0
        omega = -x - B*np.sin(phi)
        it = iter(tlist)
        tprev = it.__next__()
        yield(tprev, omega, phi, x)
        for t in it:
            dt = t - tprev
            # Verlet solver:
            phi_mid = phi + omega*dt/2
            x += A*np.sin(phi_mid)*dt
            omega = -x - B*np.sin(phi_mid)
            phi = phi_mid + omega*dt/2
            tprev = t
            yield(tprev, omega, phi, x)
    return np.array([v for v in helper()])

v = simpll(t,A=1800,B=10,omega0=140,phi0=0)
omega = v[:,1]
phi = v[:,2]

fig = plt.figure(figsize=(8,6), dpi=80)
ax = fig.add_subplot(2,1,1)
ax.plot(t,omega)
ax.set_ylabel('$\\tilde{\\omega}$',fontsize=20)
ax = fig.add_subplot(2,1,2)
ax.plot(t,phi/(2*np.pi))
ax.set_ylabel('$\\tilde{\\phi}/2\\pi$ ',fontsize=20)
ax.set_xlabel('t', fontsize=16)


'''
This is typical of the behavior of phase-locked loops: because there is no
absolute phase reference, with a large initial frequency error, you can get
cycle slips before the loop locks onto the input signal. It is often useful
to plot the behavior of phase and frequency error in phase space, rather
than as a pair of time-series plots:
'''
fig = plt.figure(figsize=(8,6), dpi=80)
ax = fig.add_subplot(1,1,1)
ax.plot(phi/(2*np.pi),omega)
ax.set_xlabel('phase error (cycles) = $\\tilde{\\phi}/2\\pi$', fontsize=16)
ax.set_ylabel('velocity error (rad/sec) = $\\tilde{\\omega}$', fontsize=16)
ax.grid('on')


# We can also try graphing a bunch of trials with different initial conditions:
fig = plt.figure(figsize=(8,6), dpi=80)
ax = fig.add_subplot(1,1,1)

t = np.linspace(0,5,2000)
for i in range(-2,2):
    for s in [-2,-1,1,2]:
        omega0 = s*100
        v = simpll(t,A=1800,B=10,omega0=omega0,phi0=(i/2.0)*np.pi)
        omega = v[:,1]
        phi = v[:,2]
        k = math.floor(phi[-1]/(2*np.pi) + 0.5)
        phi -= k*2*np.pi
        for cycrepeat in np.array([-2,-1,0,1,2])+np.sign(s):
            ax.plot(phi/(2*np.pi)+cycrepeat,omega,'k')
ax.set_ylim(-120,120)
ax.set_xlim(-1.5,1.5)
ax.set_xlabel('$\\tilde{\\phi}/2\\pi$ ',fontsize=20)
ax.set_ylabel('$\\tilde{\\omega}$ ',fontsize=20)



plt.show()

