#!/usr/bin/python


# import the soccer bot module - this will include math, time, numpy (as np) and vrep python modules
from soccerbot_lib import *

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
robotParameters.driveSystemQuality = 1		# specifies how good your drive system is from 0 to 1 (with 1 being able to drive in a perfectly straight line when a told to do so)

# Camera Parameters
robotParameters.cameraOrientation = 'landscape' # specifies the orientation of the camera, either landscape or portrait
robotParameters.cameraDistanceFromRobotCenter = 0.1 # distance between the camera and the center of the robot in the direction of the kicker/dribbler in metres
robotParameters.cameraHeightFromFloor = 0.1 # height of the camera relative to the floor in metres
robotParameters.cameraTilt = 0.0 # tilt of the camera in radians

# Vision Processing Parameters
robotParameters.maxBallDetectionDistance = 1 # the maximum distance away that you can detect the ball in metres
robotParameters.maxGoalDetectionDistance = 2.5 # the maximum distance away that you can detect the goals in metres
robotParameters.maxObstacleDetectionDistance = 1.5 # the maximum distance away that you can detect the obstacles in metres

# Dribbler Parameters
robotParameters.dribblerQuality = 1 # specifies how good your dribbler is from 0 to 1.0 (with 1.0 being awesome and 0 being non-existent)



# MAIN SCRIPT
if __name__ == '__main__':

	# Wrap everything in a try except case that catches KeyboardInterrupts and SystemExits. 
	# In the exception catch code attempt to Stop the VREP Simulator so don't have to Stop it manually
	try:

		# Create VREP SoccerBot object - this will attempt to open a connection to VREP. Make sure the VREP simulator is running.
		soccerBotSim = VREP_SoccerBot('127.0.0.1', robotParameters, sceneParameters)
		soccerBotSim.StartSimulator()


		while True:
			# move the robot at a forward velocity of 0.1m/s with a rotational velocity of 0.5 rad/s.
			soccerBotSim.SetTargetVelocities(0.1, 0, 0.5)

			# Get Detected Objects
			ballRB, blueRB, yellowRB, obstaclesRB = soccerSimBot.GetDetectedObjects()

			# Check to see if the ball is within the camera's FOV
			if ballRB != None:
				ballRange = ballRB[0]
				ballBearing = ballRB[1]

			# Check to see if any obstacles are within the camera's FOV
			if obstacles != None:
				# loop through each obstacle detected using Pythonian way
				for obstacle in obstaclesRB:
					obstacleRange = obstacle[0]
					obstacleBearing = obstacle[1]

			# Update Ball Position
			soccerBotSim.UpdateBallPosition()

	except (KeyboardInterrupt, SystemExit) as e:
		# attempt to stop simulator so it restarts and don't have to manually press the Stop button in VREP 
		soccerBotSim.StopSimulator()



