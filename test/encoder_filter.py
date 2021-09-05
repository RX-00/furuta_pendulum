import numpy as np
import matplotlib.pyplot as plt

t = np.linspace(0, 4, 5000)
pos_exact = (np.abs(t-0.5) - np.abs(t-1.5) - np.abs(t-2.5) + np.abs(t-3.5))/2
pos_measured = pos_exact + 0.04*np.random.randn(t.size)

fig = plt.figure(figsize=(8, 6),dpi=80)
ax = fig.add_subplot(1, 1, 1)
ax.plot(t,pos_measured,'.',markersize=2)
ax.plot(t,pos_exact,'k')
ax.set_ylim(-0.5, 1.5)
ax.set_title('Exact and Measured Position')
ax.set_ylabel('position')
ax.set_xlabel('time')

(-0.5, 1.5)
plt.show()
