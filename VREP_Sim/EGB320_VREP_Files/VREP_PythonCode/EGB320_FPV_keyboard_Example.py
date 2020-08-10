#!/usr/bin/python


# import the soccer bot module - this will include math, time, numpy (as np) and vrep python modules
from roverbot_lib import *
# import cv2
import pygame

#import any other required python modules


# SET SCENE PARAMETERS
sceneParameters = SceneParameters()

sceneParameters.obstacle0_StartingPosition = [-0.45, 0.5]  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
# sceneParameters.obstacle0_StartingPosition = None  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
sceneParameters.obstacle1_StartingPosition = None   # starting position of obstacle 1 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
sceneParameters.obstacle2_StartingPosition = None   # starting position of obstacle 2 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene

sceneParameters.sample0_StartingPosition = [0.5, 0]  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
# sceneParameters.sample0_StartingPosition = None  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
sceneParameters.sample1_StartingPosition = None   # starting position of obstacle 1 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
sceneParameters.sample2_StartingPosition = None   # starting position of obstacle 2 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene


# sceneParameters.rock0_StartingPosition = -1  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
sceneParameters.rock0_StartingPosition = None  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
sceneParameters.rock1_StartingPosition = None   # starting position of obstacle 1 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
sceneParameters.rock2_StartingPosition = None   # starting position of obstacle 2 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene

# SET ROBOT PARAMETERS
robotParameters = RobotParameters()

# Drive Parameters
robotParameters.driveType = 'differential'	# specify if using differential or omni drive system
robotParameters.minimumLinearSpeed = 0.0  	# minimum speed at which your robot can move forward in m/s
robotParameters.maximumLinearSpeed = 0.25 	# maximum speed at which your robot can move forward in m/s
robotParameters.driveSystemQuality = 1		# specifies how good your drive system is from 0 to 1 (with 1 being able to drive in a perfectly straight line when a told to do so)

# Camera Parameters
robotParameters.cameraOrientation = 'landscape' # specifies the orientation of the camera, either landscape or portrait
robotParameters.cameraDistanceFromRobotCenter = 0.1 # distance between the camera and the center of the robot in the direction of the kicker/dribbler in metres
robotParameters.cameraHeightFromFloor = 0.15 # height of the camera relative to the floor in metres
robotParameters.cameraTilt = 0.0 # tilt of the camera in radians

# Vision Processing Parameters
robotParameters.maxBallDetectionDistance = 1 # the maximum distance away that you can detect the ball in metres
robotParameters.maxLanderDetectionDistance = 2.5 # the maximum distance away that you can detect the goals in metres
robotParameters.maxObstacleDetectionDistance = 1.5 # the maximum distance away that you can detect the obstacles in metres

# Dribbler Parameters
robotParameters.collectorQuality = 1 # specifies how good your sample collector is from 0 to 1.0 (with 1.0 being awesome and 0 being non-existent)
robotParameters.autoCollectSample = True #specifies whether the simulator automatically collects samples if near the collector 
robotParameters.maxCollectDistance = 0.03 #specificies the operating distance of the automatic collector function. Sample needs to be less than this distance to the collector


# MAIN SCRIPT
if __name__ == '__main__':

	# Wrap everything in a try except case that catches KeyboardInterrupts. 
	# In the exception catch code attempt to Stop the VREP Simulator so don't have to Stop it manually when pressing CTRL+C
	try:

		pygame.init()
		pygame.display.set_caption("VREP Camera Stream")
		# screen = pygame.display.set_mode((400, 300))
		done = False

		# while not done:
		# 	for event in pygame.event.get():
		# 		if event.type == pygame.QUIT:
		# 			done = True
		# 	pygame.display.flip()

		# Create VREP SoccerBot object - this will attempt to open a connection to VREP. Make sure the VREP simulator is running.
		lunarBotSim = VREP_RoverRobot('127.0.0.1', robotParameters, sceneParameters)
		lunarBotSim.StartSimulator()

		image = None
		while image == None:
			resolution, image = lunarBotSim.GetCameraImage()
			
		screen = pygame.display.set_mode(resolution)
		
		#We recommended changing this to a controlled rate loop (fixed frequency) to get more reliable control behaviour
		while not done:
			# move the robot at a forward velocity of 0.2m/s with a rotational velocity of 0.3 rad/s.
			lunarBotSim.SetTargetVelocities(0.1, 0.3)

			# Get Detected Objects
			# samplesRB, landerRB, obstaclesRB, rocksRB = lunarBotSim.GetDetectedObjects()
			resolution, image = lunarBotSim.GetCameraImage()

			if image != None:

				cv2_image = np.array(image,dtype=np.uint8)
				cv2_image.resize([resolution[1],resolution[0],3])

				screen.fill([0,0,0])
				frame = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2RGB)
				frame = frame.swapaxes(0,1)
				frame = pygame.surfarray.make_surface(frame)
				screen.blit(frame, (0,0))
				pygame.display.update()

			for event in pygame.event.get():
				if event.type == pygame.QUIT:
					done = True

			# Get Detected Wall Points
			# wallPoints = lunarBotSim.GetDetectedWallPoints()
			# if wallPoints == None:
			# 	print("To close to the wall")
			# else:
			# 	print("\nDetected Wall Points")
			# 	# print the range and bearing to each wall point in the list
			# 	for point in wallPoints:
			# 		print("\tWall Point (range, bearing): %0.4f, %0.4f"%(point[0], point[1]))


			# Update Ball Position
			lunarBotSim.UpdateObjectPositions()

	except KeyboardInterrupt as e:
		# attempt to stop simulator so it restarts and don't have to manually press the Stop button in VREP 
		lunarBotSim.StopSimulator()



