#!/usr/bin/python

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
import numpy as np
import math

# SET MATPLOTLIB FONT SIZE
font = {'family' : 'normal',
        'weight' : 'normal',
        'size'   : 22}

matplotlib.rc('font', **font)

# DESIGN SPECIFICATIONS
ballMass = 0.045
# kickerVelocityInitial = 3
kickForce = 5
strokeDistance = 0.02


# IMPEDANCE MATCHING GRAPH
kickerMass = np.linspace(0, 1, 1/0.001)
ballVelocity = np.zeros(kickerMass.size)
kickerVelocityFinal = np.zeros(kickerMass.size)

for ii in range(0, kickerMass.size):
	# kickerVelocityFinal[ii] = (kickerMass[ii]-ballMass) / (kickerMass[ii] + ballMass) * kickerVelocityInitial
	ballVelocity[ii] = (2 * kickerMass[ii]) / (kickerMass[ii] + ballMass) *  math.sqrt(2 * kickForce * strokeDistance / kickerMass[ii])


fig, ax = plt.subplots()
plt.hold(True)
plt.plot(kickerMass, ballVelocity, linestyle='-', linewidth=2)
plt.title('Ball Velocity Vs. Kicker Mass')
plt.xlabel('Kicker Mass (kg)')
plt.ylabel('Ball Velocity (m/s)')
plt.hold(False)
plt.show()
