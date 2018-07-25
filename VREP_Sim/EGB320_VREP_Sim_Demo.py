#!/usr/bin/python

# Used to Import add the VREP Python Code Folder to the Python Path
import sys
sys.path.insert(0, 'EGB320_VREP_Files/VREP_PythonCode')


# import the soccer bot API - this will include math, time, random and vrep python modules
from SoccerBot_API import *

#import any other required python modules

# SET SCENE PARAMETERS
sceneParameters = SceneParameters()
sceneParameters.ballStartingPosition = [0.5, 0] # starting position of the ball [x, y] (in metres)
sceneParameters.obstacle0_StartingPosition = [0.7, 0.7]  # starting position of obstacle 1 [x, y] (in metres), or none if not wanted in the scene
sceneParameters.obstacle1_StartingPosition = None   # starting position of obstacle 1 [x, y] (in metres), or none if not wanted in the scene
sceneParameters.obstacle2_StartingPosition = None   # starting position of obstacle 1 [x, y] (in metres), or none if not wanted in the scene


# SET ROBOT PARAMETERS
robotParameters = RobotParameters()

# Drive Parameters
robotParameters.minimumLinearSpeed = 0.04  	# minimum speed at which your robot can move forward in m/s
robotParameters.maximumLinearSpeed = 0.25 	# maximum speed at which your robot can move forward in m/s
robotParameters.driveSystemQuality = 0	# specifies how good your drive system is from 0 to 1 (with 1 being able to drive in a perfectly straight line when a told to do so)

# Camera Parameters
robotParameters.cameraOrientation = 'landscape' # specifies the orientation of the camera, either landscape or portrait
robotParameters.cameraDistanceFromRobotCenter = 0.1 # distance between the camera and the center of the robot in the direction of the kicker/dribbler in metres
robotParameters.cameraHeightFromFloor = 0.03 # height of the camera relative to the floor in metres
robotParameters.cameraTilt = 0.17 # tilt of the camera in radians

# Vision Processing Parameters
robotParameters.maxBallDetectionDistance = 1 # the maximum distance away that you can detect the ball in metres
robotParameters.maxGoalDetectionDistance = 2.5 # the maximum distance away that you can detect the goals in metres
robotParameters.maxObstacleDetectionDistance = 1.5 # the maximum distance away that you can detect the obstacles in metres

# Dribbler Parameters
robotParameters.dribblerQuality = 1 # specifies how good your dribbler is from 0 to 1.0 (with 1.0 being awesome and 0 being non-existent)



# MAIN SCRIPT
if __name__ == '__main__':

	soccerBotSim = VREP_SoccerBot('127.0.0.1', robotParameters, sceneParameters)
	# soccerBotSim.SetTargetVelocities(0, 0, 0.5)

	while True:
		soccerBotSim.SetTargetVelocities(0.1,0,0)

	state = 'ball_search'
	while True:
		# check to see if ball is in dribbler
		if soccerBotSim.BallInDribbler():
			state = 'goal_search'
		else:
			state = 'ball_search'

		# get detected objects
		ballRangeBearing, blueGoalRangeBearing, yellowGoalRangeBearing, obstaclesRangeBearing = soccerBotSim.GetDetectedObjects()


		# change action depending on state
		if state == 'ball_search':
			# move towards the ball or rotate until you can see it
			if ballRangeBearing != None:
				forwardVel = 0.25*ballRangeBearing[0]
				rotationVel = 0.5*ballRangeBearing[1]

				if forwardVel < soccerBotSim.robotParameters.minimumLinearSpeed:
					forwardVel = soccerBotSim.robotParameters.minimumLinearSpeed
			else:
				forwardVel = 0
				rotationVel = 0.5

		elif state == 'goal_search':
			# either rotate to face blue goal, move towards goal, or kick
			if blueGoalRangeBearing == None:
				forwardVel = 0
				rotationVel = 0.5
			elif blueGoalRangeBearing[0] > 0.7:
				forwardVel = 0.1*blueGoalRangeBearing[0]
				rotationVel = 0.35*blueGoalRangeBearing[1]

				if forwardVel < soccerBotSim.robotParameters.minimumLinearSpeed:
					forwardVel = soccerBotSim.robotParameters.minimumLinearSpeed
			else:
				soccerBotSim.KickBall(0.25)

		# Set Velocity
		soccerBotSim.SetTargetVelocities(forwardVel, 0, rotationVel)

		# Update Ball Position
		soccerBotSim.UpdateBallPosition()



