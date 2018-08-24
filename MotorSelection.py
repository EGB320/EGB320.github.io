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
minimumVelocity = 0.2					# minimum velocity, in m/s
maximumVelocity = 1.5					# maximum velocity, in m/s
acceleration = 0.75						# minimum acceleration, in m/s^2

# ROBOT PARAMETERS
robotMass = 10							# robotMass of the robot, in kg
wheelRadius = 0.04						# radius of drive wheels, in m
wheelFriction = 0.9						# coefficient of friction between wheel and surface
efficiency = 0.8						# efficiency of the drive system

# MOTOR PARAMETERS
nominalVoltage = 24						# nominal motor voltage, in volts
stallTorque = 3  # 6						# nominal voltage stall torque, in Nm
noLoadSpeed = 450 * (2*math.pi/60.0)# 650 * (2*math.pi/60.0)	# nominal voltage no load speed, in rad/s

thresholdVoltage = 10					# voltage at which the motor will turn on
maxVoltage = 30							# maximum voltage that can send to motors




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
frictionTorque = (4.905*wheelFriction*robotMass*wheelRadius) / float(efficiency)
accelerationTorque = (0.5*robotMass*wheelRadius) * (acceleration + wheelFriction*9.81) / float(efficiency)
#accelerationTorque = (frictionTorque + wheelRadius*robotMass*acceleration) / float(efficiency) (efficiency factored in twice for friction!!)
print('Friction Torque: %f'%(frictionTorque))
print('Acceleration Torque: %f'%(accelerationTorque-frictionTorque))
print('High Motor Speed: %f'%(maxMotorSpeed))
print('Low Motor Speed: %f'%(minMotorSpeed))

# PLOT 1 - Design Requirements
fig, ax = plt.subplots()
plt.hold(True)
h1, = plt.plot([minMotorSpeed, minMotorSpeed], [0, accelerationTorque+0.2*accelerationTorque], linestyle='--', linewidth=2)
h2, = plt.plot([maxMotorSpeed, maxMotorSpeed], [0, accelerationTorque+0.2*accelerationTorque], linestyle='--', linewidth=2)
h3, = plt.plot([0, maxMotorSpeed+0.2*maxMotorSpeed], [frictionTorque, frictionTorque], linestyle='-.', linewidth=2)
h4, = plt.plot([0, maxMotorSpeed+0.2*maxMotorSpeed], [accelerationTorque, accelerationTorque], linestyle='-.', linewidth=2)

plt.legend([h1, h2, h3, h4], ['Low Motor Speed', 'High Motor Speed', 'Friction Torque', 'Acceleration Torque'], loc=0)
# plt.legend(bbox_to_anchor=(1.05, 1), , borderaxespad=0.)
plt.title('Motor Selection')
plt.xlabel('Motor Speed (rad/s)')
plt.ylabel('Torque (Nm)')
plt.hold(False)


# PLOT
# 

# determine operating zone patch
# operatingZonePoints = np.zeros([4, 2])
# operatingZonePoints[0,0] = minMotorSpeed
# operatingZonePoints[0,1] = frictionTorque
# operatingZonePoints[1,0] = minMotorSpeed
# operatingZonePoints[1,1] = lineSlope * minMotorSpeed + maxStallTorque
# operatingZonePoints[2,0] = maxMotorSpeed
# operatingZonePoints[2,1] = lineSlope * maxMotorSpeed + maxStallTorque
# operatingZonePoints[3,0] = maxMotorSpeed
# operatingZonePoints[3,1] = frictionTorque
# print operatingZonePoints
# operatingZone = Polygon(operatingZonePoints)
# patchCollections = PatchCollection([operatingZone], alpha=0.2)

# PLOT 2 - Design Requirements and Motor Specs
# plot everything
fig, ax = plt.subplots()
plt.hold(True)
h1, = plt.plot([minMotorSpeed, minMotorSpeed], [0, accelerationTorque+0.2*accelerationTorque], linestyle='--', linewidth=2)
h2, = plt.plot([maxMotorSpeed, maxMotorSpeed], [0, accelerationTorque+0.2*accelerationTorque], linestyle='--', linewidth=2)
h3, = plt.plot([0, maxMotorSpeed+0.2*maxMotorSpeed], [frictionTorque, frictionTorque], linestyle='-.', linewidth=2)
h4, = plt.plot([0, maxMotorSpeed+0.2*maxMotorSpeed], [accelerationTorque, accelerationTorque], linestyle='-.', linewidth=2)
h5, = plt.plot([0, noLoadSpeed], [stallTorque, 0], linewidth=2)
h6, = plt.plot([0, threholdNoLoadSpeed], [thresholdStallTorque, 0], linewidth=2)
#h7, = plt.plot([0, maxNoLoadSpeed], [maxStallTorque, 0], linewidth=2) # maximum voltage
plt.plot(45,4.25,'w.') #force y axis to extend up
plt.legend([h1, h2, h3, h4, h5, h6], ['Low Motor Speed', 'High Motor Speed', 'Friction Torque', 'Acceleration Torque', 'Nominal Voltage', 'Threshold Voltage'], loc=(0.25,0.52))
plt.title('Motor Selection')
plt.xlabel('Motor Speed (rad/s)')
plt.ylabel('Torque (Nm)')

# if frictionTorque < (lineSlope*minMotorSpeed+maxStallTorque):
	# ax.add_collection(patchCollections)
plt.hold(False)

plt.show()

# while True:
# 	pass