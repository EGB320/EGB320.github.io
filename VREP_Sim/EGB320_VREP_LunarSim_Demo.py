#!/usr/bin/python

# import HelperFuncs module - this sets up the robot parameters and includes helper functions
# will also include the soccerbot_lib module which includes math, time, numpy (as np) and vrep python modules
from LunarHelperFuncs import *
from lunarbot_lib import *

# SET ROBOT PARAMETERS
robotParameters = RobotParameters()

# Drive Parameters
robotParameters.driveType = 'differential'
robotParameters.minimumLinearSpeed = 0.0  	# minimum speed at which your robot can move forward in m/s
robotParameters.maximumLinearSpeed = 0.5 	# maximum speed at which your robot can move forward in m/s
robotParameters.driveSystemQuality = 1	# specifies how good your drive system is from 0 to 1 (with 1 being able to drive in a perfectly straight line when a told to do so)

# Camera Parameters
robotParameters.cameraOrientation = 'landscape' # specifies the orientation of the camera, either landscape or portrait
robotParameters.cameraDistanceFromRobotCenter = 0.1 # distance between the camera and the center of the robot in the direction of the front of the robot
robotParameters.cameraHeightFromFloor = 0.03 # height of the camera relative to the floor in metres
robotParameters.cameraTilt = 0. # tilt of the camera in radians

# Vision Processing Parameters
robotParameters.maxBallDetectionDistance = 1 # the maximum distance away that you can detect the ball in metres
robotParameters.maxGoalDetectionDistance = 2.5 # the maximum distance away that you can detect the goals in metres
robotParameters.maxObstacleDetectionDistance = 1.5 # the maximum distance away that you can detect the obstacles in metres

# Dribbler Parameters
robotParameters.dribblerQuality = 1 # specifies how good your dribbler is from 0 to 1.0 (with 1.0 being awesome and 0 being non-existent)


# SET SCENE PARAMETERS
sceneParameters = SceneParameters()
sceneParameters.ballStartingPosition = [0.3, -0.3] # starting position of the ball [x, y] (in metres)
sceneParameters.obstacle0_StartingPosition = -1  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
sceneParameters.obstacle1_StartingPosition = -1   # starting position of obstacle 1 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
sceneParameters.obstacle2_StartingPosition = -1   # starting position of obstacle 2 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene


# MAIN SCRIPT
if __name__ == '__main__':

	# Demo
	try:	
		# Create VREP_SoccerBot instance and start the simulator
		lunarBotSim = VREP_SoccerBot('127.0.0.1', robotParameters, sceneParameters)
		lunarBotSim.StartSimulator()

		# Variable to hold the x, y and rotational velocities want the robot to move 
		targetVel = [0, 0, 0]

		# Variable to hold state of the robot - tells which action to perform
		robotState = RobotStates.BALL_SEARCH_ROTATE
		previousRobotState = RobotStates.MOVE_TO_SAMPLE

		# Controller Values
		linearGain = 0.5
		rotationGain = 0.2

		# Min and Max Linear and Rotational Speeds
		linearSpeedLimits = [0.03, 0.3]
		rotationalSpeedLimits = [0.2, 0.8]

		# helper variables
		startTime = None
		searchPoint = 2

		# Figure Handles
		figHandle = None
		robotHandle = None
		landerHandle = None
		ballHandle = None
		obstacleHandles = [None, None, None]
		velocityHandle = None
		ballRBHandle = None
		landerRBHandle = None
		obstacleRBHandles = None

		while True:

			# Get object positions and transform from range bearings from camera pose to robot body pose
			ballRB, landerRB, obstaclesRB = lunarBotSim.GetDetectedObjects()
			ballRB, landerRB, obstaclesRB = TransformRangeBearingsFromCameraToRobot(robotParameters, ballRB, landerRB, obstaclesRB)

			if robotState == RobotStates.BALL_SEARCH_ROTATE:
				linearSpeedLimits = [0.03, 0.3]
				rotationalSpeedLimits = [0.1, 0.8]
				targetVel, startTime, robotState = BallSearchRotate(ballRB, lunarBotSim.BallInDribbler(), targetVel, startTime, robotState)


			elif robotState == RobotStates.BALL_SEARCH_MOVE_TO_POINT:
				linearSpeedLimits = [0.03, 0.3]
				rotationalSpeedLimits = [0.1, 0.8]
				

			elif robotState == RobotStates.MOVE_TO_SAMPLE:
				linearSpeedLimits = [0.03, 0.3]
				rotationalSpeedLimits = [0.1, 0.8]
				targetVel, robotState = MoveToBall(ballRB, obstaclesRB, lunarBotSim.BallInDribbler(), targetVel, robotState, linearGain, rotationGain, linearSpeedLimits, rotationalSpeedLimits)

			elif robotState == RobotStates.MOVE_TO_LANDER:
				linearSpeedLimits = [0.03, 0.15]
				rotationalSpeedLimits = [0.1, 0.3]
				targetVel, robotState = MoveToTarget(landerRB, obstaclesRB, lunarBotSim.BallInDribbler(), targetVel, robotState, linearGain, rotationGain, linearSpeedLimits, rotationalSpeedLimits)

			elif robotState == RobotStates.DROP_SAMPLE:
				if lunarBotSim.BallInDribbler():
					lunarBotSim.dropSample()
				else:
					robotState = RobotStates.BALL_SEARCH_ROTATE
			

			# Set Velocity and Update Ball Position
			lunarBotSim.SetTargetVelocities(targetVel[0], targetVel[1], targetVel[2])
			robotPose, ballPosition, obstaclePositions = lunarBotSim.UpdateObjectPositions()

			# move for an extra 0.5 seconds forward when range to ball is less than 0.05m
			if ballRB != None and ballRB[0] < 0.05:
				time.sleep(0.5)

			# Print state
			if previousRobotState != robotState:
				print("Robot State Changed: "),
				print(robotState),
				print("\tTarget Velocities: "),
				print(targetVel)
				previousRobotState = robotState

			# Update Plot
			figHandle, robotHandle, ballHandle, obstacleHandles = PlotArenaAndObjects(figHandle, robotHandle, ballHandle, obstacleHandles, robotPose, ballPosition, obstaclePositions)
			figHandle, ballRBHandle, obstacleRBHandles = PlotRangeAndBearings(figHandle, ballRBHandle, obstacleRBHandles, robotPose, ballRB, obstaclesRB)
			figHandle, velocityHandle = PlotTargetVelocity(figHandle, velocityHandle, targetVel, robotPose)


	except (KeyboardInterrupt) as e:
		# attempt to stop simulator so it restarts and don't have to manually press the Stop button in VREP 
		lunarBotSim.StopSimulator()



