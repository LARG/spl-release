import numpy as np
import matplotlib.pyplot as plt
import math

data = np.loadtxt("knee_info.txt")
data = np.reshape(data, newshape=(-1, 3))

command = data[:, 0]
pos = data[:,1]
interpol = data[:, 2]

def deg(x):
    return x * 180 / math.pi

plt.plot(range(len(knee_command)), deg(knee_command), 'r-', label="command")
plt.plot(range(len(knee_command)), deg(knee_pos), 'g-', label="position")
plt.plot(range(len(knee_command)), deg(knee_interpol), 'b-', label="interpoalted")
plt.legend(loc="best")
plt.show()

