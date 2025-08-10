#import any required python modules
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import math
import numpy as np
import sys
from enum import IntEnum




# This class wraps COPPELIA api functions to allow 
# users to start testing Navigation/AI systems
class warehouseObjects(IntEnum):
	
	# TODO: figure out what the enums need to be.
	bowl = 0
	mug = 1
	bottle = 2
	soccer = 3
	rubiks = 4
	cereal = 5

	obstacle0 = 6
	obstacle1 = 7
	obstacle2 = 8
	
	pickingStation = 9

	row_marker_1 = 10
	row_marker_2 = 11
	row_marker_3 = 12

	shelf_0 = 13
	shelf_1 = 14
	shelf_2 = 15
	shelf_3 = 16
	shelf_4 = 17
	shelf_5 = 18

	# Enums for turning off detection of certain objects
	items = 101
	obstacles = 102
	row_markers = 103
	shelves = 104
	#pickingStation = 105 # There's only one so it is already defined.


################################
###### WAREHOUSE BOT CLASS #####
################################


class COPPELIA_WarehouseRobot(object):
	"""docstring for COPPELIA_WarehouseRobot"""
	
	####################################
	#### COPPELIA WAREHOUSE BOT INIT ###
	####################################

	def __init__(self, coppelia_server_ip, robotParameters, sceneParameters):
		# Robot Parameters
		self.robotParameters = robotParameters
		self.leftWheelBias = 0
		self.rightWheelBias = 0

		# Scene Paramaters
		self.sceneParameters = sceneParameters

		# COPPELIA Simulator Client ID
		self.clientID = None

		# COPPELIA Object Handle Variables
		self.robotHandle = None
		self.cameraHandle = None
		self.objectDetectorHandle = None
		self.leftMotorHandle = None 		# left and right used for differential drive
		self.rightMotorHandle = None
		self.v60MotorHandle = None 			# 60, 180, 300 used for omni drive
		self.v180MotorHandle = None
		self.v300MotorHandle = None
		self.itemTemplateHandles = [None] * 6
		self.itemHandles = np.zeros((6,4,3),dtype=np.int16)
		self.obstacleHandles = [None, None, None]
		self.packingStationHandle = None
		self.rowMarkerHandles = [None,None,None]
		self.shelfHandles = [None]*6

		

		# Wheel Bias
		if self.robotParameters.driveSystemQuality != 1:
			# randomly generate a left and right wheel bias
			self.leftWheelBias = np.random.normal(0, (1-self.robotParameters.driveSystemQuality)*0.2, 1)
			self.rightWheelBias = np.random.normal(0, (1-self.robotParameters.driveSystemQuality)*0.2, 1)

		# Obstacle Parameters
		self.obstacleSize = 0.18 # diameter of obstacle

		# item Parameters
		self.itemSize = 0.05 # diameter of item

		# Variables to hold object positions
		self.robotPose = None
		self.cameraPose = None
		self.itemPositions = np.full((6,4,3,3),np.nan,dtype=np.float32)
		self.packingStationPosition = None
		self.obstaclePositions = [None, None, None]
		self.rowMarkerPositions = [None, None, None]

		# Variable to hold whether the item has been joined to the robot
		self.itemConnectedToRobot = False

		# Attempt to Open Connection to ZMQ Remote API Server
		self.OpenConnectionToZMQ(coppelia_server_ip)

		# Attempt To Get COPPELIA Object Handles
		self.GetCOPPELIAObjectHandles()

		# Send Robot Parameters to COPPELIA
		self.UpdateCOPPELIARobot()

		



	########################################
	##### WAREHOUSE BOT API FUNCTIONS ######
	########################################
	# THESE ARE THE FUNCTIONS YOU SHOULD CALL.
	# ALL OTHER FUNCTIONS ARE HELPER FUNCTIONS.

	# Starts the COPPELIA Simulator. 
	# The COPPELIA Simulator can also be started manually by pressing the Play Button in COPPELIA.
	def StartSimulator(self):
		print('Attempting to Start the Simulator')
		
		try:
			# Only set stepping mode if sync parameter is True
			if self.robotParameters.sync:
				print('Setting synchronous stepping mode (can cause hanging - consider setting sync=False)')
				self.sim.setStepping(True)
			
			self.sim.startSimulation()
			print('Successfully started the CoppeliaSim Simulator.')
		except Exception as e:
			print(f'An error occurred while trying to start the simulator: {e}')
			print('Comment out calls to StartSimulator() and start the simulator manually by pressing the Play button in CoppeliaSim.')
			sys.exit(-1)
		
		print('Setting scene')
		self.SetScene()
		
		# Note: ZMQ Remote API doesn't need streaming mode setup like legacy API
		# Object positions are retrieved directly when needed

		time.sleep(1)

		#initialise local copy of object positions
		self.GetObjectPositions()


	# Stops the COPPELIA Simulator. 
	# The CoppeliaSim Simulator can also be stopped manually by pressing the Stop Button in CoppeliaSim.
	def StopSimulator(self):
		print('Attempting to Stop the Simulator')
		try:
			self.sim.stopSimulation()
			print('Successfully stopped the CoppeliaSim Simulator.')
		except Exception as e:
			print(f'Could not stop the simulator: {e}')
			print('You can stop the simulator manually by pressing the Stop button in CoppeliaSim.')

		# Note: ZMQ Remote API doesn't need to stop streaming modes like legacy API

	# Gets the Range and Bearing to All Detected Objects.
	# returns:
	#	itemRangeBearing - range and bearing to the items with respect to the camera, will return None if the object is not detected
	#	packingStationRangeBearing - range and bearing to the picking station with respect to the camera, will return None if the object is not detected
	#	obstaclesRangeBearing - range and bearing to the obstacles with respect to the camera, will return None if the object is not detected
	def GetDetectedObjects(self,objects = None):
		import time  # Import time for internal timing
		timing_debug = hasattr(self, 'enable_timing_debug') and self.enable_timing_debug
		
		if timing_debug:
			func_start_time = time.perf_counter()
			print("🔍 [GetDetectedObjects] Starting object detection...")
		
		# Variables used to return range and bearing to the objects
		itemRangeBearing = [None]*6
		packingStationRangeBearing = None
		obstaclesRangeBearing = None
		rowMarkerRangeBearing = [None,None,None]
		shelfRangeBearing = [None]*6

		# if objects variable is None, detect all objects.
		objects= objects or [warehouseObjects.items,warehouseObjects.shelves,warehouseObjects.row_markers,warehouseObjects.obstacles,warehouseObjects.pickingStation]

		# Make sure the camera's pose is not none
		if self.cameraPose != None:

			#check which objects are currently in FOV using object detection sensor within COPPELIA sim
			try:
				if timing_debug:
					vision_start_time = time.perf_counter()
				
				# Use handleVisionSensor to get the packed detection data from Lua script
				result, data, packets = self.sim.handleVisionSensor(self.objectDetectorHandle)
				
				if timing_debug:
					vision_time = (time.perf_counter() - vision_start_time) * 1000
					print(f"🔍    handleVisionSensor call: {vision_time:.1f}ms")
				
				if result == -1:
					print("Object detector not ready or no detection")
					objectsDetected = []
				else:
					# The packets parameter contains our detection array from the Lua script!
					if packets and len(packets) > 0:
						objectsDetected = packets  # This is our 19-element detection array
						# print(f"Detection array length: {len(objectsDetected)}")
						# print(f"Detection values: {objectsDetected}")
					else:
						print("No detection packets received")
						objectsDetected = []
						
			except Exception as e:
				print(f"Error calling handleVisionSensor: {e}")
				objectsDetected = []
				
			if objectsDetected and len(objectsDetected) > 0:

				# Helper function to safely check detection array - expecting 1/0 values from Lua script
				def is_detected(obj_index):
					"""Check if object is detected (should be 1 from Lua script)"""
					return (isinstance(objectsDetected, (list, tuple)) and 
							len(objectsDetected) > obj_index and 
							obj_index >= 0 and 
							objectsDetected[obj_index] == 1)

				# Convert 1-based Lua indices to 0-based Python indices
				# Lua array: [bowl, mug, bottle, soccer, rubiks, cereal, obs0, obs1, obs2, packing_bay, row1, row2, row3, shelf0-5]
				# Python indices: 0-based, so subtract 1 from Lua positions
				
				# check to see if blue shelves are in field of view
				if warehouseObjects.shelves in objects:
					if timing_debug:
						shelf_start_time = time.perf_counter()
					
					shelfRB = self.GetShelfRangeBearing()
					
					if timing_debug:
						shelf_rb_time = (time.perf_counter() - shelf_start_time) * 1000
						print(f"🔍    GetShelfRangeBearing call: {shelf_rb_time:.1f}ms")
						shelf_process_start = time.perf_counter()
					
					for index,rb in enumerate(shelfRB):
						# Shelves are at indices 13-18 in the 19-element array (0-based: indices 13-18)
						shelf_index = 13 + index
						if is_detected(shelf_index):
							if rb and rb[0] < self.robotParameters.maxShelfDetectionDistance:
								shelfRangeBearing[index] = rb
					
					if timing_debug:
						shelf_process_time = (time.perf_counter() - shelf_process_start) * 1000
						print(f"🔍    Shelf processing: {shelf_process_time:.1f}ms")

				# check to see if item is in field of view
				if warehouseObjects.items in objects:
					if timing_debug:
						item_start_time = time.perf_counter()
					
					for shelf in range(6):
						if shelfRangeBearing[shelf] is None and warehouseObjects.shelves in objects:
							continue
						for x,y in [(x,y) for x in range(4) for y in range(3)]:
							item_type = self.sceneParameters.bayContents[shelf,x,y]
							if item_type == -1: 
								continue
							
							itemPosition = self.itemPositions[shelf,x,y]
							# Items are at indices 0-5 in the detection array (bowl, mug, bottle, soccer, rubiks, cereal)
							if is_detected(item_type)\
							and shelfRangeBearing[shelf] is not None\
							and self.PointInsideArena(itemPosition):
								_valid, _range, _bearing = self.GetRBInCameraFOV(itemPosition)
														
								# check range is not too far away
								if _range < self.robotParameters.maxItemDetectionDistance \
									and abs(_bearing) < self.robotParameters.cameraPerspectiveAngle/2:
									# make itemRangeBearing into empty lists, if currently set to None
									if itemRangeBearing[item_type] == None:
										itemRangeBearing[item_type] = []
									itemRangeBearing[item_type].append([_range, _bearing])
					
					if timing_debug:
						item_time = (time.perf_counter() - item_start_time) * 1000
						print(f"🔍    Item processing: {item_time:.1f}ms")
					
				# check to see which obstacles are within the field of view
				if warehouseObjects.obstacles in objects:
					if timing_debug:
						obstacle_start_time = time.perf_counter()
					
					for index, obstaclePosition in enumerate(self.obstaclePositions):
						if obstaclePosition != None:
							# Obstacles are at indices 6-8 in the detection array (obstacle_0, obstacle_1, obstacle_2)
							if is_detected(6 + index) and self.PointInsideArena(obstaclePosition):
								_valid, _range, _bearing = self.GetRBInCameraFOV(obstaclePosition)
								if _valid:

									# make obstaclesRangeBearing into empty lists, if currently set to None
									if obstaclesRangeBearing == None:
										obstaclesRangeBearing = []

									if _range <  self.robotParameters.maxObstacleDetectionDistance:
										obstaclesRangeBearing.append([_range, _bearing])
					
					if timing_debug:
						obstacle_time = (time.perf_counter() - obstacle_start_time) * 1000
						print(f"🔍    Obstacle processing: {obstacle_time:.1f}ms")

				# check to see if picking station is in field of view
				if warehouseObjects.pickingStation in objects:
					if timing_debug:
						packing_start_time = time.perf_counter()
					
					if self.packingStationPosition != None:
						# Picking station is at index 9 in the detection array
						if is_detected(9):
							_valid, _range, _bearing = self.GetRBInCameraFOV(self.packingStationPosition)
							if _valid:
								# check range is not to far away
								if _range < self.robotParameters.maxPackingBayDetectionDistance:
									packingStationRangeBearing = [_range, _bearing]
					
					if timing_debug:
						packing_time = (time.perf_counter() - packing_start_time) * 1000
						print(f"🔍    Packing bay processing: {packing_time:.1f}ms")

				# check to see if black row markers are in field of view
				if warehouseObjects.row_markers in objects:
					if timing_debug:
						marker_start_time = time.perf_counter()
					
					for index, rowMarkerPosition in enumerate(self.rowMarkerPositions):
						if rowMarkerPosition != None:
							# Row markers are at indices 10-12 in the detection array (row_marker1, row_marker2, row_marker3)
							if is_detected(10 + index):
								_valid, _range, _bearing = self.GetRBInCameraFOV(rowMarkerPosition)
								if _valid:
									# check range is not to far away
									if _range < self.robotParameters.maxRowMarkerDetectionDistance:
										rowMarkerRangeBearing[index] = [_range, _bearing]
					
					if timing_debug:
						marker_time = (time.perf_counter() - marker_start_time) * 1000
						print(f"🔍    Row marker processing: {marker_time:.1f}ms")

		if timing_debug:
			func_total_time = (time.perf_counter() - func_start_time) * 1000
			print(f"🔍 [GetDetectedObjects] Total time: {func_total_time:.1f}ms")

		return itemRangeBearing, packingStationRangeBearing, obstaclesRangeBearing, rowMarkerRangeBearing, shelfRangeBearing


	def GetCameraImage(self):

		if self.cameraHandle == None:
			return None, None
	
		try:
			image, resolution = self.sim.getVisionSensorImg(self.cameraHandle)
			# Unpack the image data from bytes to proper image array
			if image is not None:
				image_data = self.sim.unpackUInt8Table(image)
				return resolution, image_data
			else:
				return None, None
		except Exception as e:
			print(f"Error getting camera image: {e}")
			return None, None
	
	# Gets the Range and Bearing to the wall(s)
	# returns:
	#	None - if there are no valid wall points (i.e. the robot is right up against a wall and facing it)
	#	A list of [range, bearing] arrays. There will either be 1, 2, or 3 [range, bearing] arrays depending on the situation
	#		will return 1 range-bearing array if the robot is close to a wall but not directly facing it and one edge of the camera's view limit is up against the wall, while the other can see part of the field
	#		will return 2 range-bearing array if the robot can see the wall but is not facing a corner
	#		will return 3 range-bearing array if the robot can see the wall and is facing into a corner
	def GetDetectedWallPoints(self):
		wallPoints = None

		if self.cameraPose == None:
			return None
		
		cameraPose2D = [self.cameraPose[0], self.cameraPose[1], self.cameraPose[5]]

		# Get range and bearing to the valid points at limit of camera's view (no occlusions)
		wallPoints = self.CameraViewLimitsRangeAndBearing(cameraPose2D)
		if wallPoints == None:
			# return None to indicate to close to wall or because some maths error and didn't get 1 or 2 valid intersection points 
			# (hopefully a maths error doesn't occur and believe all cases have been taken care of)
			return None

		# See if a corner is within the field of view (no occlusions)
		cornerRangeBearing = self.FieldCornerRangeBearing(cameraPose2D)
		if cornerRangeBearing == []:
			return wallPoints

		wallPoints.append(cornerRangeBearing)
		return wallPoints
		

	# Set Target Velocities
	# inputs:
	#	x - the velocity of the robot in the forward direction (in m/s)
	#	theta_dt - the rotational velocity of the robot (in rad/s)
	def SetTargetVelocities(self, x_dot, theta_dot):
		
		# Need to set based on drive system type
		if self.robotParameters.driveType == 'differential':
			# ensure wheel base and wheel radius are set as these are not allowed to be changed
			self.robotParameters.wheelBase = 0.15
			self.robotParameters.wheelRadius = 0.04
		
			# determine minimum wheel speed based on minimumLinear and maximumLinear speed
			minWheelSpeed = self.robotParameters.minimumLinearSpeed / self.robotParameters.wheelRadius
			maxWheelSpeed = self.robotParameters.maximumLinearSpeed / self.robotParameters.wheelRadius

			# calculate left and right wheel speeds in rad/s
			leftWheelSpeed = (x_dot - 0.5*theta_dot*self.robotParameters.wheelBase) / self.robotParameters.wheelRadius + self.leftWheelBias
			rightWheelSpeed = (x_dot + 0.5*theta_dot*self.robotParameters.wheelBase) / self.robotParameters.wheelRadius + self.rightWheelBias

			# add gaussian noise to the wheel speed
			if self.robotParameters.driveSystemQuality != 1:
				leftWheelSpeed = np.random.normal(leftWheelSpeed, (1-self.robotParameters.driveSystemQuality)*1, 1)[0]
				rightWheelSpeed = np.random.normal(rightWheelSpeed, (1-self.robotParameters.driveSystemQuality)*1, 1)[0]

			# ensure wheel speeds are not greater than maximum wheel speed
			leftWheelSpeed = min(leftWheelSpeed, maxWheelSpeed)
			rightWheelSpeed = min(rightWheelSpeed, maxWheelSpeed)

			# set wheel speeds to 0 if less than the minimum wheel speed
			if abs(leftWheelSpeed) < minWheelSpeed:
				leftWheelSpeed = 0
			if abs(rightWheelSpeed) < minWheelSpeed:
				rightWheelSpeed = 0

			# Convert to float to ensure scalar values for ZMQ API
			leftWheelSpeed = float(leftWheelSpeed)
			rightWheelSpeed = float(rightWheelSpeed)

			# set motor speeds
			try:
				self.sim.setJointTargetVelocity(self.leftMotorHandle, leftWheelSpeed)
				self.sim.setJointTargetVelocity(self.rightMotorHandle, rightWheelSpeed)
				if self.leftRearMotorHandle is not None:
					self.sim.setJointTargetVelocity(self.leftRearMotorHandle, leftWheelSpeed)
				if self.rightRearMotorHandle is not None:
					self.sim.setJointTargetVelocity(self.rightRearMotorHandle, rightWheelSpeed)
			except Exception as e:
				print(f"Error setting motor velocities: {e}")

		elif self.robotParameters.driveType == 'holonomic':
			print('Holonomic base not yet implemented')

	# Returns true if the item is within the collector
	# returns:
	#	true - if item is in the collector
	def itemCollected(self):
		return self.itemConnectedToRobot

	
	def Dropitem(self):
		if self.itemConnectedToRobot:
			try:
				self.sim.callScriptFunction('RobotReleaseItem', 'Robot', [], [], [], "")
				self.itemConnectedToRobot = False
			except Exception as e:
				print(f"Error calling RobotReleaseItem script function: {e}")

	# Use this to force a physical connection between item and rover
	# Ideally use this if no collector has been added to your robot model
	# if two items are within the distance it will attempt to collect both
	# Outputs:
	#		Success - returns True if a item was collected or false if not
	def CollectItem(self,shelf_height):

		# for _, itemPosition in enumerate(self.itemPositions):
		for shelf,x,y in [(s,x,y) for s in range(6) for x in range(4) for y in range(3)]:
			handle = self.itemHandles[shelf,x,y]
			itemPosition = self.itemPositions[shelf,x,y]

			itemDist = self.CollectorToItemDistance(itemPosition)


			if itemDist != None and itemDist < self.robotParameters.maxCollectDistance and self.itemConnectedToRobot == False and self.GetItemBayHeight(itemPosition) == shelf_height:
				# make physical connection between item and robot to simulate collector
				try:
					self.sim.callScriptFunction('JoinRobotAndItem', 'Robot', [handle], [], [], "")
					self.itemConnectedToRobot = True
				except Exception as e:
					print(f"Error calling JoinRobotAndItem script function: {e}")
				
		
		return self.itemConnectedToRobot

	def GetItemBayHeight(self,itemPosition):
		if itemPosition[2] < 0.1:
			return 0
		elif itemPosition[2] < 0.2:
			return 1
		else:
			return 2

	


	# Update Object Positions - call this in every loop of your navigation code (or at the frequency your vision system runs at). 
	# This is required to get correct range and bearings to objects.
	# This function also emulates the collector. The function returns the global pose/position of the robot and the objects too. 
	# However, you should not use these return values in your nagivation code, they are there to help you debug if you wish.
	# returns: 
	#		robotPose - a 6 element array representing the robot's pose (x,y,z,roll,pitch,yaw), or None if was not successfully updated from COPPELIA
	#		itemPosition - a 3 element array representing the item's position (x,y,z), or None if was not successfully updated from COPPELIA
	#		obstaclePositions - a 3 element list, with each index in the list containing a 3 element array representing the item's position (x,y,z), or None if was not successfully updated from COPPELIA
	def UpdateObjectPositions(self):
		import time  # Import time for internal timing
		timing_debug = hasattr(self, 'enable_timing_debug') and self.enable_timing_debug
		
		if timing_debug:
			func_start_time = time.perf_counter()
			print("📍 [UpdateObjectPositions] Starting position updates...")
		
		# attempt to get object positions from COPPELIA
		if timing_debug:
			get_pos_start_time = time.perf_counter()
		
		self.GetObjectPositions()
		
		if timing_debug:
			get_pos_time = (time.perf_counter() - get_pos_start_time) * 1000
			print(f"📍    GetObjectPositions call: {get_pos_time:.1f}ms")

		# update item
		if timing_debug:
			update_item_start_time = time.perf_counter()
		
		self.UpdateItem()
		
		if timing_debug:
			update_item_time = (time.perf_counter() - update_item_start_time) * 1000
			print(f"📍    UpdateItem call: {update_item_time:.1f}ms")
			func_total_time = (time.perf_counter() - func_start_time) * 1000
			print(f"📍 [UpdateObjectPositions] Total time: {func_total_time:.1f}ms")

		# return object positions		
		return self.robotPose, self.itemPositions, self.obstaclePositions


	def readProximity(self):
		try:
			# ZMQ Remote API might return different number of values
			result = self.sim.readProximitySensor(self.proximityHandle)
			
			# Handle different return formats
			if isinstance(result, tuple) and len(result) >= 2:
				objectDetected = result[0]
				detected_point = result[1]
				if objectDetected and detected_point:
					return np.linalg.norm(detected_point)
				else:
					return float('inf')
			else:
				# If result format is unexpected, return infinite distance
				return float('inf')
				
		except Exception as e:
			print(f"Failed to read proximity sensor: {e}")
			return float('inf')  # Return infinite distance if sensor read fails

	#########################################
	####### COPPELIA API SERVER FUNCTIONS #######
	#########################################
	# These functions are called within the init function

	# Open connection to COPPELIA API Server
	# Open connection to ZMQ Remote API
	def OpenConnectionToZMQ(self, coppelia_server_ip):
		print('Attempting connection to CoppeliaSim ZMQ Remote API Server.')
		try:
			# Set a reasonable timeout to prevent hanging
			print(f'Connecting to {coppelia_server_ip}:23000...')
			self.client = RemoteAPIClient(host=coppelia_server_ip, port=23000)
			self.sim = self.client.require('sim')
			print('Connected to CoppeliaSim ZMQ Remote API Server.')
			
			# Test the connection with a simple call
			print('Testing connection...')
			simulation_time = self.sim.getSimulationTime()
			print(f'Connection test successful. Simulation time: {simulation_time}')
			
			# Check simulation state
			sim_state = self.sim.getSimulationState()
			if sim_state == self.sim.simulation_stopped:
				print('Note: Simulation is currently stopped. This is normal.')
			elif sim_state == self.sim.simulation_paused:
				print('Warning: Simulation is paused. Consider unpausing it.')
			elif sim_state == self.sim.simulation_advancing:
				print('Simulation is running.')
			
		except Exception as e:
			print(f'Failed to connect to CoppeliaSim ZMQ Remote API Server: {e}')
			print('Make sure CoppeliaSim is running with ZMQ Remote API enabled.')
			print('Also ensure the scene is loaded and simulation is not paused.')
			print('\nTroubleshooting steps:')
			print('1. Restart CoppeliaSim')
			print('2. Load your scene file')
			print('3. Try running the test_zmq_connection.py script first')
			sys.exit(-1)

	# Legacy connection method - replaced by OpenConnectionToZMQ
	def OpenConnectionToCOPPELIA(self, coppelia_server_ip):
		print("Warning: OpenConnectionToCOPPELIA is deprecated. Using ZMQ Remote API instead.")
		# Legacy code commented out as we now use ZMQ Remote API
		# # Close any open connections to coppelia in case any are still running in the background
		# print('Closing any existing COPPELIA connections.')
		# coppelia.simxFinish(-1)

		# # Attempt to connect to coppelia API server
		# print('Attempting connection to COPPELIA API Server.')
		# self.clientID = coppelia.simxStart(coppelia_server_ip, 19997, True, True, 5000, 5)
		# if self.clientID != -1:
		# 	print('Connected to COPPELIA API Server.')
		# else:
		# 	print('Failed to connect to COPPELIA API Server. Terminating Program')
		# 	sys.exit(-1)

		# if self.robotParameters.sync:
		# 	coppelia.simxSynchronous(self.clientID, True)

	def stepSim(self):
		# For sync mode, we would use sim.step() but this is not commonly used in ZMQ API
		print("Warning: stepSim is deprecated with ZMQ Remote API. Consider using async mode instead.")
		# coppelia.simxSynchronousTrigger(self.clientID)

	# Get COPPELIA Object Handles
	def GetCOPPELIAObjectHandles(self):
		# attempt to get coppelia object handles
		errorCode = self.GetRobotHandle()
		if errorCode != 0:
			print('Failed to get Robot object handle. Terminating Program. Error Code %d'%(errorCode))
			sys.exit(-1)

		errorCode = self.GetCameraHandle()
		if errorCode != 0:
			print('Failed to get Vision Sensor object handle. Terminating Program. Error Code %d'%(errorCode))
			sys.exit(-1)

		errorCode = self.GetObjectDetectorHandle()
		if errorCode != 0:
			print('Failed to get Object Detector handle. Terminating Program. Error Code %d'%(errorCode))
			sys.exit(-1)

		errorCode1, errorCode2, errorCode3,errorCode4 = self.GetMotorHandles()
		if errorCode1 != 0 or errorCode2 != 0:
			print('Failed to get Motor object handles. Terminating Program. Error Codes %d, %d, %d'%(errorCode1, errorCode2, errorCode3))
			sys.exit(-1)
		elif errorCode3 != 0 or errorCode4 != 0:
			print("Failed to get rear wheel motor handles. Disregard if using the \"Differential\" scene.")

		packingStationErrorCode = self.GetPickingStationHandle()
		if packingStationErrorCode != 0:
			print('Failed to get picking station object handles. Terminating Program. Error Codes %d'%(packingStationErrorCode))
			sys.exit(-1)

		errorCode1, errorCode2, errorCode3 = self.GetObstacleHandles()
		if errorCode1 != 0 or errorCode2 != 0 or errorCode3 != 0:
			print('Failed to get Obstacle object handles. Terminating Program. Error Codes %d, %d, %d'%(errorCode1, errorCode2, errorCode3))
			sys.exit(-1)
		

		errorCode1, errorCode2, errorCode3 = self.GetRowMarkerHandles()
		if errorCode1 != 0 or errorCode2 != 0 or errorCode3 != 0:
			print('Failed to get Row Marker object handles. Terminating Program. Error Codes %d, %d, %d'%(errorCode1, errorCode2, errorCode3))
			sys.exit(-1)

		errorCodes = self.GetItemTemplateHandles()
		if any([code != 0 for code in errorCodes]):
			print(f'Failed to get Item object handles. Terminating Program. Error Codes {errorCodes}')
			sys.exit(-1)

		errorCodes = self.getShelfHandles()
		if any([code != 0 for code in errorCodes]):
			print(f'Failed to get Shelf object handles. Terminating Program. Error Codes {errorCodes}')
			sys.exit(-1) 
	
		errorCode = self.getProximityhandle()
		if errorCode != 0:
			print(f'Failed to get Proximity sensor handle. Terminating Program. Error Codes {errorCodes}')
			sys.exit(-1) 
	
	############################################
	####### COPPELIA OBJECT HANDLE FUNCTIONS #######
	############################################
	# These functions are called by the GetCOPPELIAObjectHandles function

	# Get COPPELIA Robot Handle
	def GetRobotHandle(self):
		try:
			self.robotHandle = self.sim.getObject('/Robot')
			return 0
		except Exception as e:
			print(f"Error getting robot handle: {e}")
			return -1


	# Get ZMQ Camera Handle
	def GetCameraHandle(self):
		try:
			self.cameraHandle = self.sim.getObject('/VisionSensor')
			return 0
		except Exception as e:
			print(f"Error getting camera handle: {e}")
			return -1

	# Get ZMQ Object Detector Handle
	def GetObjectDetectorHandle(self):
		try:
			self.objectDetectorHandle = self.sim.getObject('/Robot/ObjectDetector')
			return 0
		except Exception as e:
			print(f"Error getting object detector handle: {e}")
			return -1

			
	# Get COPPELIA Motor Handles
	# Get ZMQ Motor Handles
	def GetMotorHandles(self):
		errorCode1 = 0
		errorCode2 = 0
		errorCode3 = 0
		errorCode4 = 0

		try:
			if self.robotParameters.driveType == 'differential':
				self.leftMotorHandle = self.sim.getObject('/LeftMotor')
				self.rightMotorHandle = self.sim.getObject('/RightMotor')
				try:
					self.leftRearMotorHandle = self.sim.getObject('/LeftRearMotor')
					self.rightRearMotorHandle = self.sim.getObject('/RightRearMotor')
				except:
					# Some robots may not have rear motors
					self.leftRearMotorHandle = None
					self.rightRearMotorHandle = None
		except Exception as e:
			print(f"Error getting motor handles: {e}")
			errorCode1 = -1
		
		return errorCode1, errorCode2, errorCode3, errorCode4

	# Get ZMQ Picking Station Handles
	def GetPickingStationHandle(self):
		try:
			self.packingStationHandle = self.sim.getObject('/Picking_station')
			return 0
		except Exception as e:
			print(f"Error getting picking station handle: {e}")
			return -1

	# Get ZMQ item Template Handles
	def GetItemTemplateHandles(self):
		error_codes = []
		for index, name in enumerate(["BOWL","MUG","BOTTLE","SOCCER_BALL","RUBIKS_CUBE","CEREAL_BOX"]):
			try:
				handle = self.sim.getObject(f'/{name}')
				error_codes.append(0)
				self.itemTemplateHandles[index] = handle
			except Exception as e:
				print(f"Error getting item template handle for {name}: {e}")
				error_codes.append(-1)
			
		return error_codes

	# Get ZMQ Obstacle Handles
	def GetObstacleHandles(self):
		error_codes = [0, 0, 0]
		try:
			self.obstacleHandles[0] = self.sim.getObject('/Obstacle_0')
		except:
			error_codes[0] = -1
		try:
			self.obstacleHandles[1] = self.sim.getObject('/Obstacle_1')
		except:
			error_codes[1] = -1
		try:
			self.obstacleHandles[2] = self.sim.getObject('/Obstacle_2')
		except:
			error_codes[2] = -1
		return tuple(error_codes)
	
	# Get ZMQ Row marker handles
	def GetRowMarkerHandles(self):
		error_codes = [0, 0, 0]
		try:
			self.rowMarkerHandles[0] = self.sim.getObject('/row_marker1')
		except:
			error_codes[0] = -1
		try:
			self.rowMarkerHandles[1] = self.sim.getObject('/row_marker2')
		except:
			error_codes[1] = -1
		try:
			self.rowMarkerHandles[2] = self.sim.getObject('/row_marker3')
		except:
			error_codes[2] = -1
		return tuple(error_codes)


	# Get ZMQ shelf handles
	def getShelfHandles(self):
		errorCodes = [0]*6
		for i in range(6):
			try:
				self.shelfHandles[i] = self.sim.getObject(f'/Shelf{i}')
			except:
				errorCodes[i] = -1
		return tuple(errorCodes)

	# Get ZMQ proximity sensor handle.
	def getProximityhandle(self):
		try:
			self.proximityHandle = self.sim.getObject('/Proximity_sensor')
			return 0
		except Exception as e:
			print(f"Error getting proximity sensor handle: {e}")
			return -1


	def GetShelfRangeBearing(self):
		try:
			# Use sim.checkDistance directly instead of calling script function
			results = []
			all_distance_data = []
			
			for i, shelf_handle in enumerate(self.shelfHandles):
				threshold = self.robotParameters.maxShelfDetectionDistance
				
				# Call sim.checkDistance directly - handle variable return values
				distance_result = self.sim.checkDistance(self.robotHandle, shelf_handle, threshold)
				
				# Handle different return formats from sim.checkDistance
				if isinstance(distance_result, tuple):
					if len(distance_result) >= 2:
						result = distance_result[0]
						distance_data = distance_result[1]
					else:
						result = distance_result[0] if len(distance_result) > 0 else False
						distance_data = None
				else:
					result = distance_result
					distance_data = None
				
				# Convert result to consistent error code format (0 = success)
				if result:
					results.append(0)  # Success
				else:
					results.append(1)  # Failure
				
				# Flatten distance data if it exists
				if distance_data:
					if isinstance(distance_data, (list, tuple)):
						for data_point in distance_data:
							all_distance_data.append(data_point)
					else:
						all_distance_data.append(distance_data)
				
		except Exception as e:
			print(f"Error calling sim.checkDistance: {e}")
			return [None] * 6
		
		rb = [None] * 6
		if all_distance_data:
			# Reshape distance data similar to the original implementation
			# Each shelf should have 6 data points (pA: 3 points, pB: 3 points)
			data_points_per_shelf = 6
			data_array = np.array(all_distance_data)
			
			try:
				# Reshape data to handle multiple shelves
				if len(data_array) >= len(self.shelfHandles) * data_points_per_shelf:
					data = data_array.reshape(len(self.shelfHandles), -1)
					for i, vec in enumerate(data):
						if i < len(results) and results[i] == 0:  # Success
							if len(vec) >= 6:
								pA = vec[:3]
								pB = vec[3:6]
								_, range_val, bearing = self.GetRBInCameraFOV(pB)
								rb[i] = (range_val, bearing)
				else:
					# If we don't have enough data points, try to process what we have
					print(f"Warning: Expected {len(self.shelfHandles) * data_points_per_shelf} data points, got {len(data_array)}")
					
			except Exception as reshape_error:
				print(f"Error processing distance data: {reshape_error}")
				
		return rb
	###############################################
	####### ROBOT AND SCENE SETUP FUNCTIONS #######
	###############################################
	# These functions are called within the init function

	# Updates the robot within COPPELIA based on the robot paramters
	def UpdateCOPPELIARobot(self):
		# Set Camera Pose and Orientation
		self.SetCameraPose(self.robotParameters.cameraDistanceFromRobotCenter, self.robotParameters.cameraHeightFromFloor, self.robotParameters.cameraTilt)
		self.SetCameraOrientation(self.robotParameters.cameraOrientation)

	# Sets the position of the item, robot and obstacles based on parameters
	def SetScene(self):
		print('Setting up scene objects...')
		
		# Get bay handles for positioning items (ZMQ Remote API doesn't need streaming setup)
		# bayHandles = np.zeros_like(self.sceneParameters.bayContents)
		
		# print('Attempting to get bay handles...')
		# for shelf in range(6):
		# 	for x in range(4):
		# 		for y in range(3):
		# 			bay_path = f"/Shelf{shelf}/Bay{x}{y}"
		# 			try:
		# 				print(f'Getting handle for {bay_path}')
		# 				bayHandles[shelf,x,y] = self.sim.getObject(bay_path)
		# 				print(f'Successfully got handle for {bay_path}')
		# 			except Exception as e:
		# 				print(f"Warning: Could not get bay handle for {bay_path}: {e}")
		# 				print(f"This bay may not exist in the scene - continuing...")
		# 				bayHandles[shelf,x,y] = -1  # Mark as invalid
		
		print('Bay handle retrieval completed.')
		
		# Set obstacle positions
		print('Setting obstacle positions...')
		obstacleHeight = 0.15
		for index, obstaclePosition in enumerate([self.sceneParameters.obstacle0_StartingPosition, self.sceneParameters.obstacle1_StartingPosition, self.sceneParameters.obstacle2_StartingPosition]):
			if obstaclePosition != -1:
				if obstaclePosition != None:
					coppeliaStartingPosition = [obstaclePosition[0], obstaclePosition[1], obstacleHeight/2]
					try:
						print(f'Setting obstacle {index} position to {coppeliaStartingPosition}')
						self.sim.setObjectPosition(self.obstacleHandles[index], -1, coppeliaStartingPosition)
					except Exception as e:
						print(f"Warning: Error setting obstacle {index} position: {e}")
				else:
					try:
						default_position = [2,  -0.3 + (-0.175*index), 0.8125]
						print(f'Setting obstacle {index} to default position {default_position}')
						self.sim.setObjectPosition(self.obstacleHandles[index], -1, default_position)
					except Exception as e:
						print(f"Warning: Error setting default obstacle {index} position: {e}")
		
		print('Scene setup completed.')
		
		

	### CAMERA FUNCTIONS ###

	# Sets the camera's pose
	# Inputs:
	#		x - distance between the camera and the center of the robot in the direction of the front of the robot
	#		z - height of the camera relative to the floor in metres
	#		pitch - tilt of the camera in radians
	def SetCameraPose(self, x, z, pitch):
		# assume the students want the camera in the center of the robot (so no y)
		# assume the student only wants to rotate the camera to point towards the ground or sky (so no roll or yaw)

		# update robot parameters
		self.robotParameters.cameraDistanceFromRobotCenter = x
		self.robotParameters.cameraHeightFromFloor = z
		self.robotParameters.cameraTilt = pitch

		# Need to change Z as in COPPELIA the robot frame is in the center of the Cylinder
		# z in COPPELIA robot frame = z - (cylinder height)/2 - wheel diameter
		z = z - 0.075 - 2*self.robotParameters.wheelRadius

		# Need to change the pitch by adding pi/2 (90 degrees) as pitch of 0 points up
		pitch = pitch + math.pi/2.0

		# set camera pose
		try:
			self.sim.setObjectPosition(self.cameraHandle, self.robotHandle, [x, 0, z])
			self.sim.setObjectOrientation(self.cameraHandle, self.robotHandle, [math.pi, pitch, math.pi/2.0])
		except Exception as e:
			print(f"Error setting camera pose: {e}")

	

	# Set Camera Orientation to either portrait or landscape
	def SetCameraOrientation(self, orientation):
		# get resolution based on orientation and robot parameters
		if orientation == 'portrait':
			x_res = self.robotParameters.cameraResolutionY  # swap X and Y for portrait
			y_res = self.robotParameters.cameraResolutionX
			self.verticalViewAngle = self.robotParameters.cameraPerspectiveAngle
			self.horizontalViewAngle = self.robotParameters.cameraPerspectiveAngle * x_res / y_res
		elif orientation == 'landscape':
			x_res = self.robotParameters.cameraResolutionX
			y_res = self.robotParameters.cameraResolutionY
			self.verticalViewAngle = self.robotParameters.cameraPerspectiveAngle * y_res / x_res
			self.horizontalViewAngle = self.robotParameters.cameraPerspectiveAngle
		else:
			print('The camera orientation %s is not known. You must specify either portrait or landscape')
			return


		# update robot parameters
		self.robotParameters.cameraOrientation = orientation

		# set resolution of camera (vision sensor object) - resolution parameters are int32 parameters
		try:
			self.sim.setObjectInt32Param(self.cameraHandle, self.sim.visionintparam_resolution_x, x_res)
			self.sim.setObjectInt32Param(self.cameraHandle, self.sim.visionintparam_resolution_y, y_res)
		except Exception as e:
			print(f"Error setting camera resolution: {e}")
		

	# Set Camera Resolution directly
	def SetCameraResolution(self, x_res, y_res):
		"""Set the camera resolution to specific width and height values.
		
		Args:
			x_res (int): Camera width resolution in pixels
			y_res (int): Camera height resolution in pixels
		"""
		if self.cameraHandle is None:
			print("Error: Camera handle not initialized. Cannot set resolution.")
			return False
			
		# Update robot parameters
		self.robotParameters.cameraResolutionX = x_res
		self.robotParameters.cameraResolutionY = y_res
		
		# Set resolution of camera (vision sensor object) - resolution parameters are int32 parameters
		try:
			self.sim.setObjectInt32Param(self.cameraHandle, self.sim.visionintparam_resolution_x, x_res)
			self.sim.setObjectInt32Param(self.cameraHandle, self.sim.visionintparam_resolution_y, y_res)
			print(f"Camera resolution set to {x_res}x{y_res}")
			return True
		except Exception as e:
			print(f"Error setting camera resolution: {e}")
			return False

	####################################
	####### API HELPER FUNCTIONS #######
	####################################	

	# Prints the pose/position of the objects in the scene
	def PrintObjectPositions(self):
		print("\n\n***** OBJECT POSITIONS *****")
		if self.robotPose != None:
			print("Robot 2D Pose (x,y,theta): %0.4f, %0.4f, %0.4f"%(self.robotPose[0], self.robotPose[1], self.robotPose[2]))

		if self.cameraPose != None:
			print("Camera 3D Pose (x,y,z,roll,pitch,yaw): %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f"%(self.cameraPose[0], self.cameraPose[1], self.cameraPose[2], self.cameraPose[3], self.cameraPose[4], self.cameraPose[5]))
		
		for shelf,x,y in [(s,x,y) for s in range(6) for x in range(4) for y in range(3)]:
			itemPosition = self.itemPositions[shelf,x,y]
			if np.all(np.isnan(itemPosition)) == False:
				print("item from bay [%d,%d,%d] Position (x,y,z): %0.4f, %0.4f, %0.4f"%(shelf,x,y, itemPosition[0], itemPosition[1], itemPosition[2]))
			
		if self.packingBayPosition != None:
			print("PackingBay Position (x,y,z): %0.4f, %0.4f, %0.4f"%(self.packingBayPosition[0], self.packingBayPosition[1], self.packingBayPosition[2]))
			
		for index, obstacle in enumerate(self.obstaclePositions):
			if obstacle != None:
				print("Obstacle %d Position (x,y,z): %0.4f, %0.4f, %0.4f"%(index, obstacle[0], obstacle[1], obstacle[2]))

	# Gets the pose/position in the global coordinate frame of all the objects in the scene.
	# Stores them in class variables. Variables will be set to none if could not be updated
	def GetObjectPositions(self):
		import time  # Import time for internal timing
		timing_debug = hasattr(self, 'enable_timing_debug') and self.enable_timing_debug
		
		if timing_debug:
			func_start_time = time.perf_counter()
			print("🎯 [GetObjectPositions] Starting position queries...")
		
		# Set camera pose and object position to None so can check in an error occurred
		self.robotPose = None
		self.cameraPose = None
		# self.itemPositions = [None]*len(self.itemHandles)
		self.packingStationPosition = None
		self.obstaclePositions = [None, None, None]

		# GET 2D ROBOT POSE
		try:
			if timing_debug:
				robot_start_time = time.perf_counter()
			
			robotPosition = self.sim.getObjectPosition(self.robotHandle, -1)
			robotOrientation = self.sim.getObjectOrientation(self.robotHandle, -1)
			self.robotPose = [robotPosition[0], robotPosition[1], robotPosition[1], robotOrientation[0], robotOrientation[1], robotOrientation[2]]
			
			if timing_debug:
				robot_time = (time.perf_counter() - robot_start_time) * 1000
				print(f"🎯    Robot pose query: {robot_time:.1f}ms")
		except Exception as e:
			print(f"Error getting robot pose: {e}")

		# GET 3D CAMERA POSE
		try:
			if timing_debug:
				camera_start_time = time.perf_counter()
			
			cameraPosition = self.sim.getObjectPosition(self.cameraHandle, -1)
			self.cameraPose = [cameraPosition[0], cameraPosition[1], cameraPosition[2], robotOrientation[0], robotOrientation[1], robotOrientation[2]]
			
			if timing_debug:
				camera_time = (time.perf_counter() - camera_start_time) * 1000
				print(f"🎯    Camera pose query: {camera_time:.1f}ms")
		except Exception as e:
			print(f"Error getting camera pose: {e}")
		

		# GET POSITION OF EACH OBJECT
		# for shelf,x,y in [(s,x,y) for s in range(6) for x in range(4) for y in range(3)]:
		# 	handle = self.itemHandles[shelf,x,y]
		# 	try:
		# 		itemPosition = self.sim.getObjectPosition(handle, -1)
		# 		self.itemPositions[shelf,x,y] = itemPosition
		# 	except Exception as e:
		# 		print(f"Error getting item position for shelf {shelf}, position ({x},{y}): {e}")

		# packingBay position
		try:
			if timing_debug:
				packing_start_time = time.perf_counter()
			
			packingStationPosition = self.sim.getObjectPosition(self.packingStationHandle, -1)
			self.packingStationPosition = packingStationPosition
			
			if timing_debug:
				packing_time = (time.perf_counter() - packing_start_time) * 1000
				print(f"🎯    Picking station position query: {packing_time:.1f}ms")
		except Exception as e:
			print(f"Error getting picking station position: {e}")

		# obstacle positions
		if timing_debug:
			obstacle_start_time = time.perf_counter()
		
		obstaclePositions = [None, None, None]
		for index, obs in enumerate(self.obstaclePositions):
			try:
				obstaclePositions[index] = self.sim.getObjectPosition(self.obstacleHandles[index], -1)
				self.obstaclePositions[index] = obstaclePositions[index]
			except Exception as e:
				print(f"Error getting obstacle position {index}: {e}")
		
		if timing_debug:
			obstacle_time = (time.perf_counter() - obstacle_start_time) * 1000
			print(f"🎯    Obstacle positions query (3 objects): {obstacle_time:.1f}ms")

		# row marker positions
		if timing_debug:
			marker_start_time = time.perf_counter()
		
		rowMarkerPositions = [None,None,None]
		for index, rowMarker in enumerate(self.rowMarkerPositions):
			try:
				rowMarkerPositions[index] = self.sim.getObjectPosition(self.rowMarkerHandles[index], -1)
				self.rowMarkerPositions[index] = rowMarkerPositions[index]
			except Exception as e:
				print(f"Error getting row marker position {index}: {e}")
		
		if timing_debug:
			marker_time = (time.perf_counter() - marker_start_time) * 1000
			print(f"🎯    Row marker positions query (3 objects): {marker_time:.1f}ms")
			func_total_time = (time.perf_counter() - func_start_time) * 1000
			print(f"🎯 [GetObjectPositions] Total time: {func_total_time:.1f}ms")

	# Checks to see if an Object is within the field of view of the camera
	def GetRBInCameraFOV(self, objectPosition):
		# calculate range and bearing on 2D plane - relative to the camera
		cameraPose2d = [self.cameraPose[0], self.cameraPose[1], self.cameraPose[5]]
		_range, _bearing = self.GetRangeAndBearingFromPoseAndPoint(cameraPose2d, objectPosition)

		# vertical_test_cam_pose = [0,self.cameraPose[2],0]
		# vertical_test_pos = [_range,objectPosition[2]]
		# _vert_range, _vert_bearing = self.GetRangeAndBearingFromPoseAndPoint(vertical_test_cam_pose, vertical_test_pos)
		_valid = abs(_bearing) < self.robotParameters.cameraPerspectiveAngle/2 \
		# 	and abs(_vert_bearing) < self.robotParameters.cameraPerspectiveAngle/4

		# angle from camera's axis to the object's position
		# verticalAngle = math.atan2(objectPosition[2]-self.cameraPose[2], _range)

		#OLD code needs removing

		# # check to see if in field of view
		# if abs(_bearing) > (self.horizontalViewAngle/2.0):
		# 	# return False to indicate object outside camera's FOV and range and bearing
		# 	return False, _range, _bearing

		# if abs(verticalAngle) > (self.verticalViewAngle/2.0):
		# 	# return False to indicate object outside camera's FOV and range and bearing
		# 	return False, _range, _bearing

		# return True to indicate is in FOV and range and bearing
		return _valid, _range, _bearing

	def ObjectInCameraFOV(self,objectPosition):
		_,_bearing = self.GetRBInCameraFOV(objectPosition)
		return np.abs(_bearing) <= self.robotParameters.cameraPerspectiveAngle / 2
			
	
	# Determines if a 2D point is inside the arena, returns true if that is the case
	def PointInsideArena(self, position):
		if position[0] > -1 and position[0] < 1 and position[1] > -1 and position[1] < 1:
			return True

		return False


	# Update the item
	def UpdateItem(self):
		for shelf,x,y in [(s,x,y) for s in range(6) for x in range(4) for y in range(3)]:
			itemPosition = self.itemPositions[shelf,x,y]
		
			if np.all(np.isnan(itemPosition)) == False:

				itemDist = self.CollectorToItemDistance(itemPosition)

				# DEPRACATED 
				# # See if need to connect/disconnect item from robot
				# if self.robotParameters.autoCollectItem == True and itemDist != None and itemDist < self.robotParameters.maxCollectDistance and self.itemConnectedToRobot == False:
				# 	# make physical connection between item and robot to simulate collector
				# 	coppelia.simxCallScriptFunction(self.clientID, 'Robot', coppelia.sim_scripttype_childscript, 'JoinRobotAndItem',[1],[self.robotParameters.maxCollectDistance],[],bytearray(),coppelia.simx_opmode_blocking)
				# 	self.itemConnectedToRobot = True

				if self.itemConnectedToRobot == True:
					# random chance to disconnect
					if np.random.rand() > self.robotParameters.collectorQuality:
						# terminate connection between item and robot to simulate collector
						try:
							self.sim.callScriptFunction('RobotReleaseItem', 'Robot', [], [], [], "")
							self.itemConnectedToRobot = False
						except Exception as e:
							print(f"Error calling RobotReleaseItem script function: {e}")

				elif itemDist != None and itemDist > 0.03:
					self.itemConnectedToRobot = False

	
	# Gets the range and bearing to a corner that is within the camera's field of view.
	# Will only return a single corner, as only one corner can be in the field of view with the current setup.
	# returns:
	#	a list containing a [range, bearing] or an empty list if no corner is within the field of view
	def FieldCornerRangeBearing(self, cameraPose):
		rangeAndBearing = []

		# Get range and bearing from camera's pose to each corner
		_range, _bearing = self.GetRangeAndBearingFromPoseAndPoint(cameraPose, [1, 1])
		if abs(_bearing) < (self.horizontalViewAngle/2.0):
			rangeAndBearing = [_range, _bearing]

		_range, _bearing = self.GetRangeAndBearingFromPoseAndPoint(cameraPose, [-1, 1])
		if abs(_bearing) < (self.horizontalViewAngle/2.0):
			rangeAndBearing = [_range, _bearing]

		_range, _bearing = self.GetRangeAndBearingFromPoseAndPoint(cameraPose, [-1, -1])
		if abs(_bearing) < (self.horizontalViewAngle/2.0):
			rangeAndBearing = [_range, _bearing]

		_range, _bearing = self.GetRangeAndBearingFromPoseAndPoint(cameraPose, [1, -1])
		if abs(_bearing) < (self.horizontalViewAngle/2.0):
			rangeAndBearing = [_range, _bearing]

		return rangeAndBearing


	# Gets the range and bearing to where the edge of camera's field of view intersects with the arena walls.
	# returns:
	#	None - if there are no valid wall points (i.e. the robot is right up against a wall and facing it)
	#	A list of [range, bearing] arrays. There will either be 1 or 2 [range, bearing] arrays depending on the situation
	#		will return 1 if the robot is close to a wall but not directly facing it and one edge of the camera's view limit is up against the wall, while the other can see part of the field
	#		will return 2 if the robot can see the wall but is not facing a corner
	def CameraViewLimitsRangeAndBearing(self, cameraPose):
		viewLimitIntersectionPoints = []
		rangeAndBearings = []

		# Get valid camera view limit points along the east wall
		p1, p2 = self.CameraViewLimitWallIntersectionPoints(cameraPose, 'east')
		if p1 != None:
			viewLimitIntersectionPoints.append(p1)
		if p2 != None:
			viewLimitIntersectionPoints.append(p2)

		# Get valid camera view limit points along the north wall (wall in positive y-direction)
		p1, p2 = self.CameraViewLimitWallIntersectionPoints(cameraPose, 'north')
		if p1 != None:
			viewLimitIntersectionPoints.append(p1)
		if p2 != None:
			viewLimitIntersectionPoints.append(p2)

		# Get valid camera view limit points along the west wall
		p1, p2 = self.CameraViewLimitWallIntersectionPoints(cameraPose, 'west')
		if p1 != None:
			viewLimitIntersectionPoints.append(p1)
		if p2 != None:
			viewLimitIntersectionPoints.append(p2)

		# Get valid camera view limit points along the south wall (wall in negative y-direction)
		p1, p2 = self.CameraViewLimitWallIntersectionPoints(cameraPose, 'south')
		if p1 != None:
			viewLimitIntersectionPoints.append(p1)
		if p2 != None:
			viewLimitIntersectionPoints.append(p2)

		# Calculate range and bearing to the valid view limit wall intersection points and store in a list
		for point in viewLimitIntersectionPoints:
			_range, _bearing = self.GetRangeAndBearingFromPoseAndPoint(cameraPose, point)
			rangeAndBearings.append([_range, _bearing])

		# return None if rangeAndBearings list is empty
		if rangeAndBearings == []:
			return None
		else:
			return rangeAndBearings

	
	# Gets the points where the edges of the camera's field of view intersects with the specified wall.
	# inputs:
	#	cameraPose - pose of the camera [x, y, theta] in the global coordinate frame
	# 	wall - wall want to get the camera view limit points of ('east', 'west', 'north', 'south').
	# returns:
	#	p1 - will be [x,y] point if it is a valid wall point (i.e. lies on the arena's walls and is within the field of view) or None if it is not valid
	#	p2 - will be [x,y] point if it is a valid wall point (i.e. lies on the arena's walls and is within the field of view) or None if it is not valid
	def CameraViewLimitWallIntersectionPoints(self, cameraPose, wall):
		
		# calculate range to wall along camera's axis using the point where the camera's axis intersects with the specified wall
		x, y = self.CameraViewAxisWallIntersectionPoint(cameraPose, wall)
		centreRange = math.sqrt(math.pow(cameraPose[0]-x, 2) + math.pow(cameraPose[1]-y, 2))


		# determine camera view limit intersection points on wall
		if wall == 'east' or wall == 'west':
			d1 = centreRange*math.sin(self.horizontalViewAngle/2.0) / math.sin(math.pi/2.0 - self.horizontalViewAngle/2.0 - cameraPose[2])
			d2 = centreRange*math.sin(self.horizontalViewAngle/2.0) / math.sin(math.pi/2.0 - self.horizontalViewAngle/2.0 + cameraPose[2])
		elif wall == 'north' or wall == 'south':
			d1 = centreRange*math.sin(self.horizontalViewAngle/2.0) / math.sin(math.pi - self.horizontalViewAngle/2.0 - cameraPose[2])
			d2 = centreRange*math.sin(self.horizontalViewAngle/2.0) / math.sin(cameraPose[2] - self.horizontalViewAngle/2.0)


		# add d1 and d2 (or subtract) to the camera's axis wall intersection point (add/subtract and x/y depends on wall)
		if wall == 'east' or wall == 'west':
			p1 = [x, y+d1]
			p2 = [x, y-d2]
		elif wall == 'north' or wall == 'south':
			p1 = [x-d1, y]
			p2 = [x+d2, y]

		# determine camera view limit intersection point range and bearings relative to camera
		range1, bearing1 = self.GetRangeAndBearingFromPoseAndPoint(cameraPose, p1)
		range2, bearing2 = self.GetRangeAndBearingFromPoseAndPoint(cameraPose, p2)

		# Check that the two view limit intersection points are valid (i.e. occur on the arena boundary and not outside, that the bearing is within view and the range is greater than a minimum distance)
		# Need to add small percentage to the angle due to the numerical evaluation of COPPELIA this is to ensure that after checking against all walls that 2 points are returned this is where the *1.05 comes from
		# make sure p1 is within bounds and that bearing is valid
		if (p1[0] < -1 or p1[0] > 1 or p1[1] < -1 or p1[1] > 1):
			p1 = None
		elif abs(bearing1) > (self.horizontalViewAngle/2.0)*1.05:
			p1 = None
		elif range1 < self.robotParameters.minWallDetectionDistance:
			p1 = None
		
		# make sure p2 is within bounds
		if (p2[0] < -1 or p2[0] > 1 or p2[1] < -1 or p2[1] > 1):
			p2 = None
		elif abs(bearing2) > (self.horizontalViewAngle/2.0)*1.05:
			p2 = None
		elif range2 < self.robotParameters.minWallDetectionDistance:
			p2 = None

		return p1, p2


	# Gets the point where the camera's view axis (centre of image) intersects with the specified wall.
	# inputs:
	#	cameraPose - pose of the camera [x, y, theta] in the global coordinate frame
	# 	wall - wall want to get the camera view limit points of ('east', 'west', 'north', 'south').
	# returns:
	#	x - the x coordinate where the camera's axis intersects with the specified wall
	#	y - the y coordinate where the camera's axis intersects with the specified wall
	def CameraViewAxisWallIntersectionPoint(self, cameraPose, wall):
		if wall == 'east':
			x = 1
			y = (x - cameraPose[0]) * math.tan(cameraPose[2]) + cameraPose[1]
		
		elif wall == 'north':
			y = 1
			x = (y - cameraPose[1]) / math.tan(cameraPose[2]) + cameraPose[0]

		elif wall == 'west':
			x = -1
			y = (x - cameraPose[0]) * math.tan(cameraPose[2]) + cameraPose[1]

		elif wall == 'south':
			y = -1
			x = (y - cameraPose[1]) / math.tan(cameraPose[2]) + cameraPose[0]

		return x, y
	

	# Wraps input value to be between -pi and pi
	def WrapToPi(self, radians):
		return ((radians + math.pi) % (2* math.pi) - math.pi)

	# Gets the range and bearing given a 2D pose (x,y,theta) and a point(x,y). 
	# The bearing will be relative to the pose's angle
	def GetRangeAndBearingFromPoseAndPoint(self, pose, point):
		_range = math.sqrt(math.pow(pose[0] - point[0], 2) + math.pow(pose[1] - point[1], 2))
		_bearing = self.WrapToPi(math.atan2((point[1]-pose[1]), (point[0]-pose[0])) - pose[2])

		return _range, _bearing


	# Gets the orthogonal distance (in metres) from the collector to the item. 
	# Assuming the the item's centroid is within 70 degrees of the collector's centroid
	def CollectorToItemDistance(self, itemPosition):
		# get the position of the item relative to the collector motor
		if self.robotPose != None and np.all(np.isnan(itemPosition)) == False:
			# get the pose of the collector in the x-y plane using the robot's pose with some offsets
			collectorPose = [self.robotPose[0]+0.1*math.cos(self.robotPose[5]), self.robotPose[1]+0.1*math.sin(self.robotPose[5]), self.robotPose[5]]

			# get range and bearing from collector to item position
			_range, _bearing = self.GetRangeAndBearingFromPoseAndPoint(collectorPose, itemPosition)

			# check to see if the bearing to the item is larger than 70 degrees. If so return None
			if abs(_bearing) > math.radians(70):
				return None

			# return distance to item from collector orthogonal to collector's rotational axis
			return abs(_range * math.cos(_bearing))

		return None


####################################
###### SCENE PARAMETERS CLASS ######
####################################

# This class is a helper class to simply 
# group COPPELIA scene parameters together

class SceneParameters(object):
	"""docstring for SceneParameters"""
	def __init__(self):
		# item Starting Position

		# Starting contents of the items [shelf,X,Y]. Set to -1 to leave the bay empty.
		self.bayContents = -np.ones((6,4,3),dtype=np.int16)


		# Obstacles Starting Positions - set to none if you do not want a specific obstacle in the scene
		self.obstacle0_StartingPosition = [-0.45, 0.5]  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current coppelia position, or none if not wanted in the scene
		self.obstacle1_StartingPosition = [-0.25,-0.675]   # starting position of obstacle 1 [x, y] (in metres), -1 if want to use current coppelia position, or none if not wanted in the scene
		self.obstacle2_StartingPosition = None   # starting position of obstacle 2 [x, y] (in metres), -1 if want to use current coppelia position, or none if not wanted in the scene



####################################
###### ROBOT PARAMETERS CLASS ######
####################################

# This class is a helper class to simply 
# group robot parameters together

class RobotParameters(object):
	"""docstring for RobotParameters"""
	def __init__(self):

		# Body Paramaters
		self.robotSize = 0.15 # This parameter cannot be changed
		
		# Drive/Wheel Parameters
		self.driveType = 'differential'	# specifies the drive type ('differential' is the only type currently)
		self.wheelBase = 0.150 # This parameter should not be changed
		self.wheelRadius = 0.04 # This parameter should not  be changed
		self.minimumLinearSpeed = 0.0 	# minimum speed at which your robot can move forward in m/s
		self.maximumLinearSpeed = 0.25 	# maximum speed at which your robot can move forward in m/s
		self.driveSystemQuality = 1.0 # specifies how good your drive system is from 0 to 1 (with 1 being able to drive in a perfectly straight line when a told to do so)

		# Camera Parameters
		self.cameraOrientation = 'landscape' # specifies the orientation of the camera, either landscape or portrait
		self.cameraDistanceFromRobotCenter = 0.1 # distance between the camera and the center of the robot in the direction of the front of the robot
		self.cameraHeightFromFloor = 0.1 # height of the camera relative to the floor in metres
		self.cameraTilt = 0.0 # tilt of the camera in radians
		self.cameraPerspectiveAngle = math.radians(60) # do not change this parameter
		self.cameraResolutionX = 640 # camera resolution width in pixels
		self.cameraResolutionY = 480 # camera resolution height in pixels

		# Vision Processing Parameters
		self.maxItemDetectionDistance = 1 # the maximum distance away that you can detect the item in metres
		self.maxPackingBayDetectionDistance = 2.5 # the maximum distance away that you can detect the packingBay in metres
		self.maxObstacleDetectionDistance = 1.5 # the maximum distance away that you can detect the obstacles in metres
		self.maxRowMarkerDetectionDistance = 2.5 # the maximum distance away that you can detect the obstacles in metres.
		self.maxShelfDetectionDistance = 2.5 # the maximum distance away that you can detect the shelves in metres.
		self.minWallDetectionDistance = 0.1 # the minimum distance away from a wall that you have to be to be able to detect it

		# collector Parameters
		self.collectorQuality = 1.0 # specifies how good your item collector is from 0 to 1.0 (with 1.0 being awesome and 0 being non-existent)
		self.autoCollectItem = True #specifies whether the simulator automatically collects items if near the collector 
		self.maxCollectDistance = 0.03 #specificies the operating distance of the automatic collector function. Item needs to be less than this distance to the collector

		self.sync = False

