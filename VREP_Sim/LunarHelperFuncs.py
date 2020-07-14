#!/usr/bin/python


# Used to Import add the VREP Python Code Folder to the Python Path
import sys
sys.path.insert(0, 'c:/Users/Chris/Documents/GitHub/EGB320.github.io/VREP_Sim/EGB320_VREP_Files/VREP_PythonCode')

# import the soccer bot module - this will include math, time, numpy (as np) and vrep python modules
from lunarbot_lib import *
from enum import Enum
import matplotlib.pyplot as plt
import matplotlib.patches as patches



# ROBOT STATES
class RobotStates(Enum):
	BALL_SEARCH_ROTATE = 0
	BALL_SEARCH_MOVE_TO_POINT = 1
	MOVE_TO_SAMPLE = 2
	MOVE_TO_LANDER = 3
	ATTEMPT_KICK = 4



# HELPER FUNCTIONS
def PrintObjectRangeBearings(ballRangeBearing, obstaclesRangeBearing):
	print("\n\n***** OBJECT RANGE-BEARINGS *****")

	if ballRangeBearing != None:
		print("Ball Position (r,b): %0.4f, %0.4f"%(ballRangeBearing[0], ballRangeBearing[1]))
	else:
		print("Ball Position (r,b): Not Detected")
			
	if obstaclesRangeBearing != None:
		for obstacle in obstaclesRangeBearing:
			if obstacle != None:
				print("Obstacle (r,b): %0.4f, %0.4f"%(obstacle[0], obstacle[1]))

		for ii in range(0,3-len(obstaclesRangeBearing)):
			print("Obstacle (r,b): Not Detected")
	else:
		for ii in range(0,3):
			print("Obstacle (r,b): Not Detected")


def PlotArenaAndObjects(figHandle, robotHandle, ballHandle, obstacleHandles, robotPose, ballPosition, obstaclePositions):
	if figHandle == None:
		# create figure handle
		figHandle = plt.figure(1)

		# create rectangle to represent field and add to the axis
		plt.plot([-1, 1, 1, -1, -1], [-1, -1, 1, 1, -1], color='black')
		plt.plot([-1.2, -1, -1, -1.2, -1.2], [-0.3, -0.3, 0.3, 0.3, -0.3], color='blue')
		plt.plot([1.2, 1, 1, 1.2, 1.2], [-0.3, -0.3, 0.3, 0.3, -0.3], color='yellow')

		# setup axis
		axis = plt.gca()
		axis.set_ylim([-2,2])
		plt.axis('equal')

	else:
		plt.figure(1)
		axis = plt.gca()

	# Clear previous robot, ball and obstacles
	if robotHandle != None:
		robotHandle.remove()

	if ballHandle != None:
		ballHandle.remove()

	for handle in obstacleHandles:
		if handle != None:
			handle.remove()

	# plot robot onto current axis
	if ballPosition != None:
		robotHandle = patches.Circle((robotPose[0], robotPose[1]), 0.09, color='b')
		axis.add_patch(robotHandle)

	# plot ball onto current axis
	if ballPosition != None:
		ballHandle = patches.Circle((ballPosition[0], ballPosition[1]), 0.025, color='r')
		axis.add_patch(ballHandle)

	# plot obstacles onto current axis
	for index, obstaclePos in enumerate(obstaclePositions):
		if obstaclePos != None:
			obstacleHandles[index] = patches.Circle((obstaclePos[0], obstaclePos[1]), 0.09, color='black')
			axis.add_patch(obstacleHandles[index])
		else:
			obstacleHandles[index] = None

	# show the plot and return the handle
	plt.draw()
	plt.pause(0.001)
	return figHandle, robotHandle, ballHandle, obstacleHandles


def PlotRangeAndBearings(figHandle, ballRBHandle, obstacleRBHandles, robotPose, ballRB, obstaclesRB):
	if figHandle == None:
		# create figure handle
		figHandle = plt.figure(1)

		# create rectangle to represent field and add to the axis
		plt.plot([-1, 1, 1, -1, -1], [-1, -1, 1, 1, -1], color='black')
		plt.plot([-1.2, -1, -1, -1.2, -1.2], [-0.3, -0.3, 0.3, 0.3, -0.3], color='blue')
		plt.plot([1.2, 1, 1, 1.2, 1.2], [-0.3, -0.3, 0.3, 0.3, -0.3], color='yellow')

		# setup axis
		axis = plt.gca()
		axis.set_ylim([-2,2])
		plt.axis('equal')

	else:
		plt.figure(1)
		axis = plt.gca()


	# Clear lines if possible
	if ballRBHandle != None and ballRBHandle != []:
		ballRBHandle.pop(0).remove()

	if obstacleRBHandles != None:
		for handle in obstacleRBHandles:
			handle.pop(0).remove()
	obstacleRBHandles = None


	# Plot lines
	if ballRB != None:
		x = [robotPose[0], robotPose[0]+ballRB[0]*math.cos(ballRB[1]+robotPose[5])]
		y = [robotPose[1], robotPose[1]+ballRB[0]*math.sin(ballRB[1]+robotPose[5])]
		ballRBHandle = plt.plot(x, y, '--r')

	if obstaclesRB != None:
		obstacleRBHandles = []
		for obsRB in obstaclesRB:
			x = [robotPose[0], robotPose[0]+obsRB[0]*math.cos(obsRB[1]+robotPose[5])]
			y = [robotPose[1], robotPose[1]+obsRB[0]*math.sin(obsRB[1]+robotPose[5])]
			handle = plt.plot(x, y, '--k')
			obstacleRBHandles.append(handle)


	return figHandle, ballRBHandle, obstacleRBHandles


def PlotTargetVelocity(figHandle, velocityHandle, targetVel, robotPose):
	if figHandle == None:
		# create figure handle
		figHandle = plt.figure(1)

		# create rectangle to represent field and add to the axis
		plt.plot([-1, 1, 1, -1, -1], [-1, -1, 1, 1, -1], color='black')
		plt.plot([-1.2, -1, -1, -1.2, -1.2], [-0.3, -0.3, 0.3, 0.3, -0.3], color='blue')
		plt.plot([1.2, 1, 1, 1.2, 1.2], [-0.3, -0.3, 0.3, 0.3, -0.3], color='yellow')

		# setup axis
		axis = plt.gca()
		axis.set_ylim([-2,2])
		plt.axis('equal')

	else:
		plt.figure(1)
		axis = plt.gca()


	# Clear velocity handle
	if velocityHandle != None:
		velocityHandle.pop(0).remove()


	# get magnitude and bearing of target velocity in global coordinates
	bearing = math.atan2(targetVel[1], targetVel[0]) + robotPose[5]
	mag = math.sqrt(math.pow(targetVel[0], 2) + math.pow(targetVel[1], 2))

	x = [robotPose[0], robotPose[0] + 3*mag*math.cos(bearing)]
	y = [robotPose[1],  robotPose[1] + 3*mag*math.sin(bearing)]

	velocityHandle = plt.plot(x, y, 'g')

	return figHandle, velocityHandle


def TransformRangeBearingsFromCameraToRobot(robotParameters, ballRangeBearing, obstaclesRangeBearing):
	if ballRangeBearing != None:
		x = ballRangeBearing[0]*math.cos(ballRangeBearing[1]) + robotParameters.cameraDistanceFromRobotCenter
		y = ballRangeBearing[0]*math.sin(ballRangeBearing[1])
		ballRangeBearing[0] = math.sqrt(math.pow(x, 2) + math.pow(y, 2))
		ballRangeBearing[1] = math.atan2(y,x)

	if obstaclesRangeBearing != None:
		for idx, obsRB in enumerate(obstaclesRangeBearing):
			x = obsRB[0]*math.cos(obsRB[1]) + robotParameters.cameraDistanceFromRobotCenter
			y = obsRB[0]*math.sin(obsRB[1])
			obstaclesRangeBearing[idx][0] = math.sqrt(math.pow(x, 2) + math.pow(y, 2))
			obstaclesRangeBearing[idx][1] = math.atan2(y,x)

	return ballRangeBearing, obstaclesRangeBearing


def BallSearchRotate(ballRB, ballInDribbler, targetVel, startTime, robotState):
	# check to make sure ball is not in the dribbler
	if ballInDribbler:
		robotState = RobotStates.MOVE_TO_LANDER
		targetVel = [0, 0, 0]
		startTime = None
		return targetVel, startTime, robotState 

	# Check to see if ball is in FOV
	if ballRB != None:
		# ball is in view change state to move towards ball
		robotState = RobotStates.MOVE_TO_SAMPLE
		targetVel = [0, 0, 0]
		startTime = None
		return targetVel, startTime, robotState 

	
	# Check to see if need to start timer. If already started check to see if rotated 360 degrees (based on time)
	if startTime == None:
		# start time and set velocit
		startTime = time.time()
		targetVel = [0, 0, 0.4]

	elif (time.time() - startTime) > (2*math.pi / targetVel[2]):
		# rotated more than 360 degrees (approximately) change state to move towards next point
		robotState = RobotStates.BALL_SEARCH_MOVE_TO_POINT
		targetVel = [0, 0, 0]
		startTime = None

	return targetVel, startTime, robotState 


def MoveToBall(ballRB, obstaclesRB, ballInDribbler, targetVel, robotState, linearGain, rotationalGain, linearSpeedLimits, rotationalSpeedLimits):
	# check to see if ball is in dribbler
	if ballInDribbler:
		robotState = RobotStates.MOVE_TO_LANDER
		targetVel = [linearSpeedLimits[0], 0, 0]
		return targetVel, robotState

	# Check to see if ball is still in FOV
	if ballRB == None:
		# ball is in view change state to move towards ball
		robotState = RobotStates.BALL_SEARCH_ROTATE
		targetVel = [0, 0, 0.4]
		return targetVel, robotState


	# get repulsive vector
	vectorComponents = ComputeRepulsiveVectorComponents(obstaclesRB)

	# add on attractive component
	vectorComponents[0] = ballRB[0]*math.cos(ballRB[1]) - vectorComponents[0]
	vectorComponents[1] = ballRB[0]*math.sin(ballRB[1]) - vectorComponents[1]

	# vector in terms of magnitude and direction
	vector = [0, 0]
	vector[0] = math.sqrt(math.pow(vectorComponents[0], 2) + math.pow(vectorComponents[1], 2))
	vector[1] = math.atan2(vectorComponents[1], vectorComponents[0])


	# determine target velocity
	targetVel[2] = min(rotationalSpeedLimits[1], max(-rotationalSpeedLimits[1], vector[1]*rotationalGain))

	linearVelocityScale = linearSpeedLimits[1] * (1 - 0.8 * abs(targetVel[2])/rotationalSpeedLimits[1])
	targetVel[0] = max(linearGain * linearVelocityScale * vectorComponents[0], linearSpeedLimits[0])
	targetVel[1] = linearGain * linearVelocityScale * vectorComponents[1]

	return targetVel, robotState


def MoveToTarget(targetRB, obstaclesRB, ballInDribbler, targetVel, robotState, linearGain, rotationalGain, linearSpeedLimits, rotationalSpeedLimits):
	
    if ballInDribbler == False:
		robotState = RobotStates.BALL_SEARCH_ROTATE
		targetVel = [0, 0, 0.4]
		return targetVel, robotState


	if targetRB == None:
		targetVel = [0, 0, 0.2]
		return targetVel, robotState

    #logic for changing into kick state
	# if abs(targetRB[1]) < math.radians(10) and targetRB[0] < 0.7:
	# 	robotState = RobotStates.ATTEMPT_KICK
	# 	return targetVel, robotState


	# get repulsive vector
	vectorComponents = ComputeRepulsiveVectorComponents(obstaclesRB)

	# add on attractive component
	vectorComponents[0] = vectorComponents[0] + targetRB[0]*math.cos(targetRB[1])
	vectorComponents[1] = vectorComponents[1] + targetRB[0]*math.sin(targetRB[1])

	# vector in terms of magnitude and direction
	vector = [0, 0]
	vector[0] = math.sqrt(math.pow(vectorComponents[0], 2) + math.pow(vectorComponents[1], 2))
	vector[1] = math.atan2(vectorComponents[1], vectorComponents[0])


	# determine target velocity
	targetVel[2] = min(rotationalSpeedLimits[1], max(-rotationalSpeedLimits[1], vector[1]*rotationalGain))

	linearVelocityScale = linearSpeedLimits[1] * (1 - 0.8 * abs(targetVel[2])/rotationalSpeedLimits[1])
	targetVel[0] = max(linearGain * linearVelocityScale * vectorComponents[0], linearSpeedLimits[0])
	targetVel[1] = linearGain * linearVelocityScale * vectorComponents[1]

	return targetVel, robotState


def ComputeRepulsiveVectorComponents(obstaclesRB):
	repulsiveVector = [0, 0]

	if obstaclesRB == None:
		return repulsiveVector

	for obsRB in obstaclesRB:
		# compute the x and y components scale based on the range
		repulsiveVector[0] = repulsiveVector[0] + obsRB[0]*math.cos(obsRB[1])
		repulsiveVector[1] = repulsiveVector[1] + obsRB[0]*math.sin(obsRB[1])

	return repulsiveVector