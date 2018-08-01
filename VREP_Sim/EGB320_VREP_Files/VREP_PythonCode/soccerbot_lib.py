#import any required python modules
import vrep
import time
import math
import numpy as np


################################
####### SOCCER BOT CLASS #######
################################

# This class wraps VREP api functions to allow 
# users to start testing Navigation/AI systems


class VREP_SoccerBot(object):
	"""docstring for VREP_SoccerBot"""
	
	####################################
	####### VREP SOCCER BOT INIT #######
	####################################

	def __init__(self, vrep_server_ip, robotParameters, sceneParameters):
		# Robot Parameters
		self.robotParameters = robotParameters
		self.leftWheelBias = 0
		self.rightWheelBias = 0

		# Scene Paramaters
		self.sceneParameters = sceneParameters

		# VREP Simulator Client ID
		self.clientID = None

		# VREP Object Handle Variables
		self.robotHandle = None
		self.cameraHandle = None
		self.leftMotorHandle = None
		self.rightMotorHandle = None
		self.dribblerMotorHandle = None
		self.kickerHandle = None
		self.ballHandle = None
		self.obstacleHandles = [None, None, None]
		self.blueGoalHandle = None
		self.yellowGoalHandle = None

		# Wheel Bias
		if self.robotParameters.driveSystemQuality != 1:
			# randomly generate a left and right wheel bias
			self.leftWheelBias = np.random.normal(0, (1-self.robotParameters.driveSystemQuality)*0.2, 1)
			self.rightWheelBias = np.random.normal(0, (1-self.robotParameters.driveSystemQuality)*0.2, 1)

		# Obstacle Parameters
		self.obstacleSize = 0.18 # diameter of obstacle

		# Ball Parameters
		self.ballSize = 0.05 # diameter of ball

		# Attempt to Open Connection to VREP API Server
		self.OpenConnectionToVREP(vrep_server_ip)

		# Attempt To Get VREP Object Handles
		self.GetVREPObjectHandles()

		# Send Robot Parameters to VREP and set the scene
		self.UpdateVREPRobot()
		self.SetScene()



	########################################
	####### SOCCER BOT API FUNCTIONS #######
	########################################
	# THESE ARE THE FUNCTIONS YOU SHOULD CALL.
	# ALL OTHER FUNCTIONS ARE HELPER FUNCTIONS.

	# Starts the VREP Simulator. 
	# The VREP Simulator can also be started manually by pressing the Play Button in VREP.
	def StartSimulator(self):
		print('Attempting to Start the Simulator')
		if vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_oneshot_wait) != 0:
			print('An error occurred while trying to start the simulator via the Python API. Terminating Program!')
			print('Comment out calls to StartSimulator() and start the simulator manully by pressing the Play button in VREP.')
			exit(-1)
		else:
			print('Successfully started the VREP Simulator.')


	# Stops the VREP Simulator. 
	# The VREP Simulator can also be stopped manually by pressing the Stop Button in VREP.
	def StopSimulator(self):
		print('Attempting to Stop the Simulator')
		if vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_oneshot_wait) != 0:
			print('Could not stop the simulator. You can stop the simulator manually by pressing the Stop button in VREP.')
		else:
			print('Successfully stoped the VREP Simulator.')


	# Gets the Range and Bearing to All Detected Objects.
	# returns:
	#	ballRangeBearing - range and bearing to the ball with respect to the camera, will return None if the object is not detected
	#	blueGoalRangeBearing - range and bearing to the blue goal with respect to the camera, will return None if the object is not detected
	#	yellowGoalRangeBearing - range and bearing to the yellow goal with respect to the camera, will return None if the object is not detected
	#	obstaclesRangeBearing - range and bearing to the obstacles with respect to the camera, will return None if the object is not detected
	def GetDetectedObjects(self):
		ballRangeBearing = None
		blueGoalRangeBearing = None
		yellowGoalRangeBearing = None
		obstaclesRangeBearing = None


		# check to see if ball is in field of view
		inFOV, _range, _bearing = self.ObjectInCameraFOV(self.ballHandle, self.robotParameters.maxBallDetectionDistance)
		if inFOV == True:
			ballRangeBearing = [_range, _bearing]

		# check to see if either goal is in field of view
		inFOV, _range, _bearing = self.ObjectInCameraFOV(self.blueGoalHandle, self.robotParameters.maxGoalDetectionDistance)
		if inFOV == True:
			blueGoalRangeBearing = [_range, _bearing]

		inFOV, _range, _bearing = self.ObjectInCameraFOV(self.yellowGoalHandle, self.robotParameters.maxGoalDetectionDistance)
		if inFOV == True:
			yellowGoalRangeBearing = [_range, _bearing]

		# check to see if any obstacles are in field of view
		for ii in range(0, 3):
			inFOV, _range, _bearing = self.ObjectInCameraFOV(self.obstacleHandles[ii], self.robotParameters.maxObstacleDetectionDistance)
			if inFOV == True and self.ObstacleOutsideArena(ii) == False:
				if obstaclesRangeBearing == None:
					obstaclesRangeBearing = []
				obstaclesRangeBearing.append([_range, _bearing])

		return ballRangeBearing, blueGoalRangeBearing, yellowGoalRangeBearing, obstaclesRangeBearing
	
	# Set Target Velocities
	# inputs:
	#	x - the velocity of the robot in the forward direction (in m/s)
	#	y - the velocity of the robot in the direction orthogonal to the forward direction  (in m/s) (only used for omni systems)
	#	theta_dt - the rotational velocity of the robot (in rad/s)
	def SetTargetVelocities(self, x_dot, y_dot, theta_dot):
		# ensure wheel base and wheel radius are set as these are not allowed to be changed
		self.robotParameters.wheelBase = 0.16
		self.robotParameters.wheelRadius = 0.025

		# determine minimum wheel speed based on minimumLinear and maximumLinear speed
		minWheelSpeed = self.robotParameters.minimumLinearSpeed / self.robotParameters.wheelRadius
		maxWheelSpeed = self.robotParameters.maximumLinearSpeed / self.robotParameters.wheelRadius

		# calculate left and right wheel speeds in rad/s
		leftWheelSpeed = (x_dot - 0.5*theta_dot*self.robotParameters.wheelBase) / self.robotParameters.wheelRadius + self.leftWheelBias
		rightWheelSpeed = (x_dot + 0.5*theta_dot*self.robotParameters.wheelBase) / self.robotParameters.wheelRadius + self.rightWheelBias

		# add gaussian noise to the wheel speed
		if self.robotParameters.driveSystemQuality != 1:
			leftWheelSpeed = np.random.normal(leftWheelSpeed, (1-self.robotParameters.driveSystemQuality)*1, 1)
			rightWheelSpeed = np.random.normal(rightWheelSpeed, (1-self.robotParameters.driveSystemQuality)*1, 1)

		# ensure wheel speeds are not greater than maximum wheel speed
		leftWheelSpeed = min(leftWheelSpeed, maxWheelSpeed)
		rightWheelSpeed = min(rightWheelSpeed, maxWheelSpeed)

		# set wheel speeds to 0 if less than the minimum wheel speed
		if abs(leftWheelSpeed) < minWheelSpeed:
			leftWheelSpeed = 0
		if abs(rightWheelSpeed) < minWheelSpeed:
			rightWheelSpeed = 0

		# set motor speeds
		errorCode = vrep.simxSetJointTargetVelocity(self.clientID, self.leftMotorHandle, leftWheelSpeed, vrep.simx_opmode_oneshot_wait)
		errorCode = vrep.simxSetJointTargetVelocity(self.clientID, self.rightMotorHandle, rightWheelSpeed, vrep.simx_opmode_oneshot_wait) 
		if errorCode != 0:
			print('Failed to set left and/or right motor speed. Error code %d'%errorCode) 

	# Returns true if the ball is within the dribbler
	# returns:
	#	true - if ball is in the dribbler
	def BallInDribbler(self):
		errorCode, position = vrep.simxGetObjectPosition(self.clientID, self.ballHandle, self.dribblerMotorHandle, vrep.simx_opmode_oneshot_wait)
		if math.sqrt(math.pow(position[0], 2) + math.pow(position[1], 2)) < 0.033:
			return True

		return False

	# Will attempt to fire the kicker plate. The kick plate will not be fired if the kicker plate
	# has not reset itself (will reset automatically with time, takes approximately 1 second).
	# inputs:
	#	kickSpeed - the velocity of the kicker
	def KickBall(self, kickSpeed):
		# check to make sure kicker has reset
		errorCode, jointPosition = vrep.simxGetJointPosition(self.clientID, self.kickerHandle, vrep.simx_opmode_oneshot_wait)
		if errorCode == 0 and jointPosition < 0.005:
			# kick and wait short time to reset kicker position
			vrep.simxSetJointTargetVelocity(self.clientID, self.kickerHandle, kickSpeed, vrep.simx_opmode_oneshot_wait)
			vrep.simxSetJointPosition(self.clientID, self.kickerHandle, 0.04, vrep.simx_opmode_oneshot_wait)
			time.sleep(0.04/kickSpeed)

			# reset kicker position
			vrep.simxSetJointTargetVelocity(self.clientID, self.kickerHandle, -0.05, vrep.simx_opmode_oneshot_wait)
			vrep.simxSetJointPosition(self.clientID, self.kickerHandle, 0, vrep.simx_opmode_oneshot_wait)


	# Update Ball Position - call this in every loop as to reset the ball position if in the goal
	# this function also emulates your dribbler quality
	def UpdateBallPosition(self):
		errorCode, position = vrep.simxGetObjectPosition(self.clientID, self.ballHandle, -1, vrep.simx_opmode_oneshot_wait)
		if errorCode == 0 and position[0] > -1 and position[0] < 1 and position[1] > -1 and position[1] < 1:
			# random chance that the ball will come out of dribbler, to do this let us slow the dribbler down
			if self.robotParameters.dribblerQuality == 0:
				errorCode = vrep.simxSetJointTargetVelocity(self.clientID, self.dribblerMotorHandle, 0, vrep.simx_opmode_oneshot_wait)
			elif np.random.rand() < (1 - self.robotParameters.dribblerQuality):
				errorCode = vrep.simxSetJointTargetVelocity(self.clientID, self.dribblerMotorHandle, 0, vrep.simx_opmode_oneshot_wait)
				time.sleep(0.05)
				errorCode = vrep.simxSetJointTargetVelocity(self.clientID, self.dribblerMotorHandle, 87.2, vrep.simx_opmode_oneshot_wait)

		else:
			# ball is in a goal, lets reset it to the center of the arena
			vrep.simxSetObjectPosition(self.clientID, self.ballHandle, -1, [0,0,0.725], vrep.simx_opmode_oneshot_wait)




	#########################################
	####### VREP API SERVER FUNCTIONS #######
	#########################################
	# These functions are called within the init function

	# Open connection to VREP API Server
	def OpenConnectionToVREP(self, vrep_server_ip):
		# Close any open connections to vrep in case any are still running in the background
		print('Closing any existing VREP connections.')
		vrep.simxFinish(-1)

		# Attempt to connect to vrep API server
		print('Attempting connection to VREP API Server.')
		self.clientID = vrep.simxStart(vrep_server_ip, 19997, True, True, 5000, 5)
		if self.clientID != -1:
			print('Connected to VREP API Server.')
		else:
			print('Failed to connect to VREP API Server. Terminating Program')
			exit(-1)


	# Get VREP Object Handles
	def GetVREPObjectHandles(self):
		# attempt to get vrep object handles
		errorCode = self.GetRobotHandle()
		if errorCode != 0:
			print('Failed to get Robot object handle. Terminating Program.')
			exit(-1)

		errorCode = self.GetCameraHandle()
		if errorCode != 0:
			print('Failed to get Vision Sensor object handle. Terminating Program.')
			exit(-1)

		leftErrorCode, rightErrorCode = self.GetMotorHandles()
		if leftErrorCode != 0 or rightErrorCode != 0:
			print('Failed to get Motor object handles. Terminating Program.')
			exit(-1)

		errorCode = self.GetdribblerMotorHandle()
		if errorCode != 0:
			print('Failed to get Dribbler object handle. Terminating Program.')
			exit(-1)

		errorCode = self.GetkickerHandle()
		if errorCode != 0:
			print('Failed to get Kicker object handle. Terminating Program.')
			exit(-1)

		errorCode = self.GetBallHandle()
		if errorCode != 0:
			print('Failed to get Ball object handle. Terminating Program.')
			exit(-1)

		blueErrorCode, yellowErrorCode = self.GetGoalHandles()
		if blueErrorCode != 0 or yellowErrorCode != 0:
			print('Failed to get Motor object handles. Terminating Program.')
			exit(-1)

		obs0ErrorCode, obs1ErrorCode, obs2ErrorCode = self.GetObstacleHandles()
		if obs0ErrorCode != 0 or obs1ErrorCode != 0 or obs2ErrorCode != 0:
			print('Failed to get Obstacle object handles. Terminating Program.')
			exit(-1)


	############################################
	####### VREP OBJECT HANDLE FUNCTIONS #######
	############################################
	# These functions are called by the GetVREPObjectHandles function

	# Get VREP Robot Handle
	def GetRobotHandle(self):
		errorCode, self.robotHandle = vrep.simxGetObjectHandle(self.clientID, 'Robot', vrep.simx_opmode_oneshot_wait)
		return errorCode


	# Get VREP Camera Handle
	def GetCameraHandle(self):
		errorCode, self.cameraHandle = vrep.simxGetObjectHandle(self.clientID, 'VisionSensor', vrep.simx_opmode_oneshot_wait)
		return errorCode

			
	# Get VREP Motor Handles
	def GetMotorHandles(self):
		leftErrorCode, self.leftMotorHandle = vrep.simxGetObjectHandle(self.clientID, 'LeftMotor', vrep.simx_opmode_oneshot_wait)
		rightErrorCode, self.rightMotorHandle = vrep.simxGetObjectHandle(self.clientID, 'RightMotor', vrep.simx_opmode_oneshot_wait)
		return leftErrorCode, rightErrorCode

			
	# Get VREP Dribbler Handle
	def GetdribblerMotorHandle(self):
		errorCode, self.dribblerMotorHandle = vrep.simxGetObjectHandle(self.clientID, 'DribblerMotor', vrep.simx_opmode_oneshot_wait)
		return errorCode
			

	# Get VREP Kicker Handle
	def GetkickerHandle(self):
		errorCode, self.kickerHandle = vrep.simxGetObjectHandle(self.clientID, 'Kicker', vrep.simx_opmode_oneshot_wait)
		return errorCode


	# Get VREP Goal Handles
	def GetGoalHandles(self):
		blueErrorCode, self.blueGoalHandle = vrep.simxGetObjectHandle(self.clientID, 'BlueGoal', vrep.simx_opmode_oneshot_wait)
		yellowErrorCode, self.yellowGoalHandle = vrep.simxGetObjectHandle(self.clientID, 'YellowGoal', vrep.simx_opmode_oneshot_wait)	
		return blueErrorCode, yellowErrorCode


	# Get VREP Ball Handle
	def GetBallHandle(self):
		errorCode, self.ballHandle = vrep.simxGetObjectHandle(self.clientID, 'Ball', vrep.simx_opmode_oneshot_wait)
		return errorCode


	# Get VREP Obstacle Handles
	def GetObstacleHandles(self):
		obs0ErrorCode, self.obstacleHandles[0] = vrep.simxGetObjectHandle(self.clientID, 'Obstacle_0', vrep.simx_opmode_oneshot_wait)
		obs1ErrorCode, self.obstacleHandles[1] = vrep.simxGetObjectHandle(self.clientID, 'Obstacle_1', vrep.simx_opmode_oneshot_wait)
		obs2ErrorCode, self.obstacleHandles[2] = vrep.simxGetObjectHandle(self.clientID, 'Obstacle_2', vrep.simx_opmode_oneshot_wait)
		return obs0ErrorCode, obs1ErrorCode, obs2ErrorCode

	###############################################
	####### ROBOT AND SCENE SETUP FUNCTIONS #######
	###############################################
	# These functions are called within the init function

	# Updates the robot within VREP based on the robot paramters
	def UpdateVREPRobot(self):
		# Set Camera Pose and Orientation
		self.SetCameraPose(self.robotParameters.cameraDistanceFromRobotCenter, self.robotParameters.cameraHeightFromFloor, self.robotParameters.cameraTilt)
		self.SetCameraOrientation(self.robotParameters.cameraOrientation)

	# Sets the position of the ball, robot and obstacles based on parameters
	def SetScene(self):
		
		# move ball to starting position
		vrepStartingPosition = [self.sceneParameters.ballStartingPosition[0], self.sceneParameters.ballStartingPosition[1], 0.725]
		vrep.simxSetObjectPosition(self.clientID, self.ballHandle, -1, vrepStartingPosition, vrep.simx_opmode_oneshot_wait)
		
		# move obstacle 0 to starting position
		if self.sceneParameters.obstacle0_StartingPosition != None:
			vrepStartingPosition = [self.sceneParameters.obstacle0_StartingPosition[0], self.sceneParameters.obstacle0_StartingPosition[1], 0.8125]
			vrep.simxSetObjectPosition(self.clientID, self.obstacleHandles[0], -1, vrepStartingPosition, vrep.simx_opmode_oneshot_wait)
			
		# move obstacle 1 to starting position
		if self.sceneParameters.obstacle1_StartingPosition != None:
			vrepStartingPosition = [self.sceneParameters.obstacle1_StartingPosition[0], self.sceneParameters.obstacle1_StartingPosition[1], 0.8125]
			vrep.simxSetObjectPosition(self.clientID, self.obstacleHandles[1], -1, vrepStartingPosition, vrep.simx_opmode_oneshot_wait)
			
		# move obstacle 2 to starting position
		if self.sceneParameters.obstacle2_StartingPosition != None:
			vrepStartingPosition = [self.sceneParameters.obstacle2_StartingPosition[0], self.sceneParameters.obstacle2_StartingPosition[1], 0.8125]
			vrep.simxSetObjectPosition(self.clientID, self.obstacleHandles[2], -1, vrepStartingPosition, vrep.simx_opmode_oneshot_wait)
			

	### CAMERA FUNCTIONS ###

	# Sets the camera's pose
	# Inputs:
	#		x - distance between the camera and the center of the robot in the direction of the kicker/dribbler in metres
	#		z - height of the camera relative to the floor in metres
	#		pitch - tilt of the camera in radians
	def SetCameraPose(self, x, z, pitch):
		# assume the students want the camera in the center of the robot (so no y)
		# assume the student only wants to rotate the camera to point towards the ground or sky (so no roll or yaw)

		# update robot parameters
		self.robotParameters.cameraDistanceFromRobotCenter = x
		self.robotParameters.cameraHeightFromFloor = z
		self.robotParameters.cameraTilt = pitch

		# Need to change Z as in VREP the robot frame is in the center of the Cylinder
		# z in VREP robot frame = z - (cylinder height)/2 - wheel diameter
		z = z - 0.075 - 2*self.robotParameters.wheelRadius

		# Need to change the pitch by adding pi/2 (90 degrees) as pitch of 0 points up
		pitch = pitch + math.pi/2.0

		# set camera pose
		vrep.simxSetObjectPosition(self.clientID, self.cameraHandle, vrep.sim_handle_parent, [x,0,z], vrep.simx_opmode_oneshot_wait)
		vrep.simxSetObjectOrientation(self.clientID, self.cameraHandle, vrep.sim_handle_parent, [0,pitch,math.pi/2.0], vrep.simx_opmode_oneshot_wait)

	
	# Sets the camera's height relative to the floor in metres
	def SetCameraHeight(self, z):
		self.SetCameraPose(0, z, 0)


	# Sets the distance between the camera and the center of the robot in the direction of the kicker/dribbler in metres
	def SetCameraOffsetFromRobotCentre(self, x):
		self.SetCameraPose(x, 0, 0)


	# Sets the tilt of the camera in radians
	def SetCameraTilt(self, pitch):
		self.SetCameraPose(0, 0, pitch)


	# Set Camera Orientation to either portrait or landscape
	def SetCameraOrientation(self, orientation):
		# get resolution based on orientation
		if orientation == 'portrait':
			x_res = 480
			y_res = 640
			self.verticalViewAngle = self.robotParameters.cameraPerspectiveAngle
			self.horizontalViewAngle = self.robotParameters.cameraPerspectiveAngle * x_res / y_res
		elif orientation == 'landscape':
			x_res = 640
			y_res = 480
			self.verticalViewAngle = self.robotParameters.cameraPerspectiveAngle * y_res / x_res
			self.horizontalViewAngle = self.robotParameters.cameraPerspectiveAngle
		else:
			print('The camera orientation %s is not known. You must specify either portrait or landscape')
			return


		# update robot parameters
		self.robotParameters.cameraOrientation = orientation

		# set resolution of camera (vision sensor object) - resolution parameters are int32 parameters
		vrep.simxSetObjectIntParameter(self.clientID, self.cameraHandle, vrep.sim_visionintparam_resolution_x, x_res, vrep.simx_opmode_oneshot_wait)
		vrep.simxSetObjectIntParameter(self.clientID, self.cameraHandle, vrep.sim_visionintparam_resolution_y, y_res, vrep.simx_opmode_oneshot_wait)
		

	####################################
	####### API HELPER FUNCTIONS #######
	####################################	

	# Checks to see if an Object is within the field of view of the camera
	def ObjectInCameraFOV(self, objectHandle, maxViewDistance):
		# get object position relative to the camera
		errorCode, position = vrep.simxGetObjectPosition(self.clientID, objectHandle, self.cameraHandle, vrep.simx_opmode_oneshot_wait)
		if errorCode != 0:
			# return False to indicate object could not be found
			return errorCode, 0, 0

		# calculate range, horizontal and vertical angles
		_range = position[2]
		horizontalAngle = math.atan2(position[0], position[2])
		verticalAngle = math.atan2(position[1], position[2])

		# check range is not to far away
		if _range > maxViewDistance:
			return False, 0, 0

		# UNTESTED CODE
		

		# check to see if in field of view
		if abs(horizontalAngle) > (self.horizontalViewAngle/2.0):
			# return False to indicate object could not be found
			return False, 0, 0

		if abs(verticalAngle) > (self.verticalViewAngle/2.0):
			# return False to indicate object could not be found
			return False, 0, 0

		# return 0 to indicate is in FOV and range and horizontalAngle as the bearing
		return True, _range, horizontalAngle

	
	# Determines if an obstacle is outside the arena, returns true if that is the case
	def ObstacleOutsideArena(self, obstacleID):
		errorCode, position = vrep.simxGetObjectPosition(self.clientID, self.obstacleHandles[obstacleID], -1, vrep.simx_opmode_oneshot_wait)
		if errorCode == 0 and position[0] > -1 and position[0] < 1 and position[1] > -1 and position[1] < 1:
			return False

		return True



####################################
###### SCENE PARAMETERS CLASS ######
####################################

# This class is a helper class to simply 
# group VREP scene parameters together

class SceneParameters(object):
	"""docstring for SceneParameters"""
	def __init__(self):
		# Ball Starting Position
		self.ballStartingPosition = [0.5, 0] # starting position of the ball [x, y] (in metres)

		# Obstacles Starting Positions - set to none if you do not want a specific obstacle in the scene
		self.obstacle0_StartingPosition = [0.7, 0.7]  # starting position of obstacle 1 [x, y] (in metres), or none if not wanted in the scene
		self.obstacle1_StartingPosition = None   # starting position of obstacle 1 [x, y] (in metres), or none if not wanted in the scene
		self.obstacle2_StartingPosition = None   # starting position of obstacle 1 [x, y] (in metres), or none if not wanted in the scene


####################################
###### ROBOT PARAMETERS CLASS ######
####################################

# This class is a helper class to simply 
# group robot parameters together

class RobotParameters(object):
	"""docstring for RobotParameters"""
	def __init__(self):

		# Body Paramaters
		self.robotSize = 0.18 # This parameter cannot be changed
		
		# Drive/Wheel Parameters
		self.wheelBase = 0.160 # This parameter cannot be changed
		self.wheelRadius = 0.025 # This parameter cannot be changed
		self.minimumLinearSpeed = 0.04 	# minimum speed at which your robot can move forward in m/s
		self.maximumLinearSpeed = 0.25 	# maximum speed at which your robot can move forward in m/s
		self.driveSystemQuality = 1.0 # specifies how good your drive system is from 0 to 1 (with 1 being able to drive in a perfectly straight line when a told to do so)

		# Camera Parameters
		self.cameraOrientation = 'landscape' # specifies the orientation of the camera, either landscape or portrait
		self.cameraDistanceFromRobotCenter = 0.1 # distance between the camera and the center of the robot in the direction of the kicker/dribbler in metres
		self.cameraHeightFromFloor = 0.1 # height of the camera relative to the floor in metres
		self.cameraTilt = 0.0 # tilt of the camera in radians
		self.cameraPerspectiveAngle = math.radians(60) # do not change this parameter

		# Vision Processing Parameters
		self.maxBallDetectionDistance = 1 # the maximum distance away that you can detect the ball in metres
		self.maxGoalDetectionDistance = 2.5 # the maximum distance away that you can detect the goals in metres
		self.maxObstacleDetectionDistance = 1.5 # the maximum distance away that you can detect the obstacles in metres

		# Dribbler Parameters
		self.dribblerQuality = 1.0 # specifies how good your dribbler is from 0 to 1.0 (with 1.0 being awesome and 0 being non-existent)
