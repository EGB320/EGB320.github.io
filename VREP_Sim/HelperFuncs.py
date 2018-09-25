#!/usr/bin/python


# Used to Import add the VREP Python Code Folder to the Python Path
import sys
sys.path.insert(0, 'EGB320_VREP_Files/VREP_PythonCode')

# import the soccer bot module - this will include math, time, numpy (as np) and vrep python modules
from soccerbot_lib import *
from enum import Enum
import matplotlib.pyplot as plt
import matplotlib.patches as patches


# SET ROBOT PARAMETERS
robotParameters = RobotParameters()

# Drive Parameters
robotParameters.driveType = 'omni'
robotParameters.minimumLinearSpeed = 0.0  	# minimum speed at which your robot can move forward in m/s
robotParameters.maximumLinearSpeed = 0.5 	# maximum speed at which your robot can move forward in m/s
robotParameters.driveSystemQuality = 1	# specifies how good your drive system is from 0 to 1 (with 1 being able to drive in a perfectly straight line when a told to do so)

# Camera Parameters
robotParameters.cameraOrientation = 'landscape' # specifies the orientation of the camera, either landscape or portrait
robotParameters.cameraDistanceFromRobotCenter = 0.1 # distance between the camera and the center of the robot in the direction of the kicker/dribbler in metres
robotParameters.cameraHeightFromFloor = 0.03 # height of the camera relative to the floor in metres
robotParameters.cameraTilt = 0. # tilt of the camera in radians

# Vision Processing Parameters
robotParameters.maxBallDetectionDistance = 1 # the maximum distance away that you can detect the ball in metres
robotParameters.maxGoalDetectionDistance = 2.5 # the maximum distance away that you can detect the goals in metres
robotParameters.maxObstacleDetectionDistance = 1.5 # the maximum distance away that you can detect the obstacles in metres

# Dribbler Parameters
robotParameters.dribblerQuality = 1 # specifies how good your dribbler is from 0 to 1.0 (with 1.0 being awesome and 0 being non-existent)


# ROBOT STATES
class RobotStates(Enum):
	BALL_SEARCH_ROTATE = 0
	BALL_SEARCH_MOVE_TO_POINT = 1
	MOVE_TO_BALL = 2
	MOVE_TO_GOAL = 3
	ATTEMPT_KICK = 4



# HELPER FUNCTIONS
def PrintObjectRangeBearings(ballRangeBearing, blueGoalRangeBearing, yellowGoalRangeBearing, obstaclesRangeBearing):
	print("\n\n***** OBJECT RANGE-BEARINGS *****")

	if ballRangeBearing != None:
		print("Ball Position (r,b): %0.4f, %0.4f"%(ballRangeBearing[0], ballRangeBearing[1]))
	else:
		print("Ball Position (r,b): Not Detected")
			
	if blueGoalRangeBearing != None:
		print("Blue Goal Position (r,b): %0.4f, %0.4f"%(blueGoalRangeBearing[0], blueGoalRangeBearing[1]))
	else:
		print("Blue Goal Position (r,b): Not Detected")
		
	if yellowGoalRangeBearing != None:
		print("Yellow Goal Position (r,b): %0.4f, %0.4f"%(yellowGoalRangeBearing[0], yellowGoalRangeBearing[1]))
	else:
		print("Yellow Goal Position (r,b): Not Detected")

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


def PlotRangeAndBearings(figHandle, ballRBHandle, blueGoalRBHandle, yellowGoalRBHandle, obstacleRBHandles, robotPose, ballRB, blueGoalRB, yellowGoalRB, obstaclesRB):
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

	if blueGoalRBHandle != None and blueGoalRBHandle != []:
		blueGoalRBHandle.pop(0).remove()

	if yellowGoalRBHandle != None and yellowGoalRBHandle != []:
		yellowGoalRBHandle.pop(0).remove()

	if obstacleRBHandles != None:
		for handle in obstacleRBHandles:
			handle.pop(0).remove()
	obstacleRBHandles = None



	# Plot lines
	if ballRB != None:
		x = [robotPose[0], robotPose[0]+ballRB[0]*math.cos(ballRB[1]+robotPose[5])]
		y = [robotPose[1], robotPose[1]+ballRB[0]*math.sin(ballRB[1]+robotPose[5])]
		ballRBHandle = plt.plot(x, y, '--r')

	if blueGoalRB != None:
		x = [robotPose[0], robotPose[0]+blueGoalRB[0]*math.cos(blueGoalRB[1]+robotPose[5])]
		y = [robotPose[1], robotPose[1]+blueGoalRB[0]*math.sin(blueGoalRB[1]+robotPose[5])]
		blueGoalRBHandle = plt.plot(x, y, '--b')

	if yellowGoalRB != None:
		x = [robotPose[0], robotPose[0]+yellowGoalRB[0]*math.cos(yellowGoalRB[1]+robotPose[5])]
		y = [robotPose[1], robotPose[1]+yellowGoalRB[0]*math.sin(yellowGoalRB[1]+robotPose[5])]
		yellowGoalRBHandle = plt.plot(x, y, '--y')


	if obstaclesRB != None:
		obstacleRBHandles = []
		for obsRB in obstaclesRB:
			x = [robotPose[0], robotPose[0]+obsRB[0]*math.cos(obsRB[1]+robotPose[5])]
			y = [robotPose[1], robotPose[1]+obsRB[0]*math.sin(obsRB[1]+robotPose[5])]
			handle = plt.plot(x, y, '--k')
			obstacleRBHandles.append(handle)


	return figHandle, ballRBHandle, blueGoalRBHandle, yellowGoalRBHandle, obstacleRBHandles


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



def BallSearchRotate(ballRB, ballInDribbler, targetVel, startTime, robotState):
	# check to make sure ball is not in the dribbler
	if ballInDribbler:
		robotState = RobotStates.MOVE_TO_GOAL
		targetVel = [0, 0, 0]
		startTime = None
		return targetVel, startTime, robotState 

	# Check to see if ball is in FOV
	if ballRB != None:
		# ball is in view change state to move towards ball
		robotState = RobotStates.MOVE_TO_BALL
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
		robotState = RobotStates.MOVE_TO_GOAL
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



def MoveToGoal(goalRB, obstaclesRB, ballInDribbler, targetVel, robotState, linearGain, rotationalGain, linearSpeedLimits, rotationalSpeedLimits):
	if ballInDribbler == False:
		robotState = RobotStates.BALL_SEARCH_ROTATE
		targetVel = [0, 0, 0.4]
		return targetVel, robotState


	if goalRB == None:
		targetVel = [0, 0, 0.2]
		return targetVel, robotState


	if abs(goalRB[1]) < math.radians(10) and goalRB[0] < 0.7:
		robotState = RobotStates.ATTEMPT_KICK
		return targetVel, robotState


	# get repulsive vector
	vectorComponents = ComputeRepulsiveVectorComponents(obstaclesRB)

	# add on attractive component
	vectorComponents[0] = vectorComponents[0] + goalRB[0]*math.cos(goalRB[1])
	vectorComponents[1] = vectorComponents[1] + goalRB[0]*math.sin(goalRB[1])

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