#!/usr/bin/python

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
import numpy as np
import math

# DESIGN SPECIFICATIONS
minimumVelocity = 0.03					# minimum velocity, in m/s
maximumVelocity = 0.35					# maximum velocity, in m/s
acceleration = 0.35						# minimum acceleration, in m/s^2

# ROBOT PARAMETERS
mass = 1								# mass of the robot, in kg
wheelRadius = 0.02						# radius of drive wheels, in m
wheelBase = 0.18						# wheel base of the robot, in m
wheelFriction = 0.8						# coefficient of friction between wheel and surface

# MOTOR PARAMETERS
nominalVoltage = 6						# nominal motor voltage, in volts
stallTorque = 24 * 0.0070615518333333	# nominal voltage stall torque, in Nm
noLoadSpeed = 310 * (2*math.pi/60.0)	# nominal voltage no load speed, in rad/s

thresholdVoltage = 3					# voltage at which the motor will turn on
maxVoltage = 7.4						# maximum voltage that can send to motors




# CALCULATIONS
# threshold and max motor curves
lineSlope = -1 * float(stallTorque) / float(noLoadSpeed)
thresholdStallTorque = thresholdVoltage/float(nominalVoltage) * stallTorque
threholdNoLoadSpeed = thresholdVoltage/float(nominalVoltage) * noLoadSpeed
maxStallTorque = maxVoltage/float(nominalVoltage) * stallTorque
maxNoLoadSpeed = maxVoltage/float(nominalVoltage) * noLoadSpeed

# minimum and maximum speed lines
minMotorSpeed = minimumVelocity / float(wheelRadius)
maxMotorSpeed = maximumVelocity / float(wheelRadius)

# torque to overcome friction and torque required for acceleration
frictionTorque = 4.905*wheelFriction*mass*wheelRadius
accelerationTorque = frictionTorque + wheelRadius*mass*acceleration 


# PLOT
# determine operating zone patch
operatingZonePoints = np.zeros([4, 2])
operatingZonePoints[0,0] = minMotorSpeed
operatingZonePoints[0,1] = frictionTorque
operatingZonePoints[1,0] = minMotorSpeed
operatingZonePoints[1,1] = lineSlope * minMotorSpeed + maxStallTorque
operatingZonePoints[2,0] = maxMotorSpeed
operatingZonePoints[2,1] = lineSlope * maxMotorSpeed + maxStallTorque
operatingZonePoints[3,0] = maxMotorSpeed
operatingZonePoints[3,1] = frictionTorque
print operatingZonePoints
operatingZone = Polygon(operatingZonePoints)
patchCollections = PatchCollection([operatingZone], alpha=0.3)

# plot everything
fig, ax = plt.subplots()
plt.hold(True)
h1, = plt.plot([0, noLoadSpeed], [stallTorque, 0])
h2, = plt.plot([0, threholdNoLoadSpeed], [thresholdStallTorque, 0])
h3, = plt.plot([0, maxNoLoadSpeed], [maxStallTorque, 0])
h4, = plt.plot([minMotorSpeed, minMotorSpeed], [0, maxStallTorque], '--')
h5, = plt.plot([maxMotorSpeed, maxMotorSpeed], [0, maxStallTorque], '--')
h6, = plt.plot([0, maxNoLoadSpeed], [frictionTorque, frictionTorque], '.-')
h7, = plt.plot([0, maxNoLoadSpeed], [accelerationTorque, accelerationTorque], '.-')

if frictionTorque < (lineSlope*minMotorSpeed+maxStallTorque):
	ax.add_collection(patchCollections)
plt.hold(False)

plt.title('Motor Selection')
plt.xlabel('Motor Speed (rad/s)')
plt.ylabel('Torque (Nm)')
plt.legend([h1, h2, h3, h4, h5, h6], ['Nominal Voltage', 'Threshold Voltage', 'Max Voltage', 'Min Speed', 'Max Speed', 'Friction Torque', 'Acceleration Torque'])
plt.show()

# while True:
# 	pass