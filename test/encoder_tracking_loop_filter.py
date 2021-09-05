import numpy as np
import matplotlib.pyplot as plt

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


# Tracking Loop estimator that attempts to follow the input
# by using an error and a feedback loop to try to drive the
# error to zero (similar to PLL but w/o oscillation)
def track_loop(x, dt, kp, ki):

    def helper():
        vel_est = 0
        pos_est = 0
        vel_integrator = 0

        for x_meas in x:
            pos_est += vel_est * dt
            pos_err = x_meas - pos_est
            vel_integrator += pos_err * ki * dt
            vel_est = pos_err * kp + vel_integrator
            yield (pos_est, vel_est, vel_integrator)

    y = np.array([yi for yi in helper()])
    return y[:,0], y[:,1], y[:,2]

[pos_est, vel_est, vel_est_filt] = track_loop(pos_measured, t[1]-t[0], kp=40.0, ki=900.0)

fig = plt.figure(figsize=(8, 6), dpi=80)
ax = fig.add_subplot(2, 1, 1)
ax.plot(t, pos_exact, 'k', t, pos_est)
ax.set_ylabel('position')

ax = fig.add_subplot(2, 1, 2)
err = pos_est - pos_exact
ax.plot(t, pos_est - pos_exact)
ax.set_ylabel('position error')
ax.set_xlabel('time')
print( 'rms error = %.5f, peak error = %.4f' % (rms(err), maxabs(err)))


# Plot the two estimates of velocity
# -> Velocity Integrator of the PI loop used in tracking loop (blue)
# -> Velocity output of the PI loop (yellow)
# reference velocity exact in red
fig = plt.figure(figsize=(8,6),dpi=80)
ax = fig.add_subplot(1,1,1)
vel_exact = (t > 0.5) * (t < 1.5) + (-1.0*(t > 2.5) * (t < 3.5))
ax.plot(t, vel_est, 'y', t, vel_est_filt, 'b', t, vel_exact, 'r');
ax.set_ylabel('velocity')
ax.set_xlabel('time')


plt.show()

