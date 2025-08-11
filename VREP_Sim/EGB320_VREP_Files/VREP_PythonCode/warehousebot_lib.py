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
	pickingStation1 = 19
	pickingStation2 = 20
	pickingStation3 = 21

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
	PickingStationMarkers = 105  # All picking station markers (individual picking stations)
	#pickingStation = 105 # There's only one so it is already defined.


################################
###### WAREHOUSE BOT CLASS #####
################################


class COPPELIA_WarehouseRobot(object):
	"""docstring for COPPELIA_WarehouseRobot"""
	
	####################################
	#### COPPELIA WAREHOUSE BOT INIT ###
	####################################

	def __init__(self, robotParameters, sceneParameters, coppelia_server_ip='127.0.0.1', port=23000):
		# Robot Parameters
		self.robotParameters = robotParameters
		self.leftWheelBias = 0
		self.rightWheelBias = 0

		# Scene Paramaters
		self.sceneParameters = sceneParameters
		
		# Store port number for ZMQ connection
		self.port = port

		# COPPELIA Simulator Client ID
		self.clientID = None

		# COPPELIA Object Handle Variables
		self.robotHandle = None
		self.scriptHandle = None  # Handle for the script attached to the Robot object
		self.cameraHandle = None
		self.objectDetectorHandle = None
		self.collectorForceSensorHandle = None  # Handle for the CollectorForceSensor
		self.leftMotorHandle = None 		# left and right used for differential drive
		self.rightMotorHandle = None
		self.v60MotorHandle = None 			# 60, 180, 300 used for omni drive
		self.v180MotorHandle = None
		self.v300MotorHandle = None
		self.itemTemplateHandles = [None] * 6
		self.itemHandles = np.zeros((6,4,3),dtype=np.int16)
		self.obstacleHandles = [None, None, None]
		self.packingStationHandle = None
		self.pickingStationHandles = [None, None, None]  # Handles for picking stations 1, 2, 3
		self.pickingStationItemHandles = [None, None, None]  # Handles for items at picking stations 1, 2, 3
		self.rowMarkerHandles = [None,None,None]
		self.shelfHandles = [None]*6
		self.bayHandles = np.full((6,4,3), None, dtype=object)  # Handles for bay dummies [shelf][x][y]

		

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
		
		# Handle of the currently held item (for direct API release)
		self.heldItemHandle = None

		# Attempt to Open Connection to ZMQ Remote API Server
		self.OpenConnectionToZMQ(coppelia_server_ip, self.port)

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

	def _is_object_detected(self, objectsDetected, obj_index):
		"""Helper function to safely check detection array - expecting 1/0 values from Lua script"""
		return (isinstance(objectsDetected, (list, tuple)) and 
				len(objectsDetected) > obj_index and 
				obj_index >= 0 and 
				objectsDetected[obj_index] == 1)
	
	def _process_single_object_detection(self, position, detection_index, objectsDetected, max_detection_distance):
		"""Helper function to process detection of a single object at a position"""
		if position is not None and self._is_object_detected(objectsDetected, detection_index):
			if self.PointInsideArena(position):
				_valid, _range, _bearing = self.GetRBInCameraFOV(position)
				if _valid and _range < max_detection_distance:
					return [_range, _bearing]
		return None
	
	def _process_multiple_object_detection(self, positions, start_index, objectsDetected, max_detection_distance):
		"""Helper function to process detection of multiple objects with sequential indices"""
		results = []
		for index, position in enumerate(positions):
			result = self._process_single_object_detection(position, start_index + index, objectsDetected, max_detection_distance)
			results.append(result)
		return results
	
	def _add_item_to_range_bearing(self, itemRangeBearing, item_type, range_bearing):
		"""Helper function to add item detection to range bearing list"""
		if itemRangeBearing[item_type] is None:
			itemRangeBearing[item_type] = []
		itemRangeBearing[item_type].append(range_bearing)

	# Gets the Range and Bearing to All Detected Objects.
	# returns:
	#	itemRangeBearing - range and bearing to the items with respect to the camera, will return None if the object is not detected
	#	packingStationRangeBearing - range and bearing to the main picking station with respect to the camera, will return None if the object is not detected
	#	obstaclesRangeBearing - range and bearing to the obstacles with respect to the camera, will return None if the object is not detected
	#	rowMarkerRangeBearing - range and bearing to the row markers with respect to the camera, will return None if the object is not detected
	#	shelfRangeBearing - range and bearing to the shelves with respect to the camera, will return None if the object is not detected
	#	pickingStationRangeBearing - range and bearing to individual picking stations 1,2,3 with respect to the camera, will return None if the object is not detected
	def GetDetectedObjects(self,objects = None):
		# Variables used to return range and bearing to the objects
		itemRangeBearing = [None]*6
		packingStationRangeBearing = None
		obstaclesRangeBearing = None
		rowMarkerRangeBearing = [None,None,None]
		shelfRangeBearing = [None]*6
		pickingStationRangeBearing = [None, None, None]  # Individual picking stations 1, 2, 3

		# if objects variable is None, detect all objects.
		objects= objects or [warehouseObjects.items,warehouseObjects.shelves,warehouseObjects.row_markers,warehouseObjects.obstacles,warehouseObjects.pickingStation,warehouseObjects.PickingStationMarkers]

		# Make sure the camera's pose is not none
		if self.cameraPose != None:

			#check which objects are currently in FOV using object detection sensor within COPPELIA sim
			try:
				# Use handleVisionSensor to get the packed detection data from Lua script
				result, data, packets = self.sim.handleVisionSensor(self.objectDetectorHandle)
				
				if result == -1:
					print("Object detector not ready or no detection")
					objectsDetected = []
				else:
					# The packets parameter contains our detection array from the Lua script!
					if packets and len(packets) > 0:
						objectsDetected = packets  # This is our 23-element detection array
						# print(f"Detection array length: {len(objectsDetected)}")
						# print(f"Detection values: {objectsDetected}")
					else:
						print("No detection packets received")
						objectsDetected = []
						
			except Exception as e:
				print(f"Error calling handleVisionSensor: {e}")
				objectsDetected = []
				
			if objectsDetected and len(objectsDetected) > 0:

				# Convert 1-based Lua indices to 0-based Python indices
				# Lua array: [bowl, mug, bottle, ball, rubiks, cereal, obstacle_0, obstacle_1, obstacle_2, packing_bay, row_marker1, row_marker2, row_marker3, shelf0-5, square_marker1-3, picking_station]
				# New 23-element array indices:
				# 0: bowl, 1: mug, 2: bottle, 3: ball, 4: rubiks, 5: cereal
				# 6: obstacle_0, 7: obstacle_1, 8: obstacle_2
				# 9: packing_bay
				# 10: row_marker1, 11: row_marker2, 12: row_marker3  
				# 13-18: shelf0-5
				# 19-21: square_marker1-3 (picking stations 1-3)
				# 22: picking_station
				
				# check to see if blue shelves are in field of view
				if warehouseObjects.shelves in objects:
					shelfRB = self.GetShelfRangeBearing()
					
					for index,rb in enumerate(shelfRB):
						# Shelves are at indices 13-18 in the 19-element array (0-based: indices 13-18)
						shelf_index = 13 + index
						if self._is_object_detected(objectsDetected, shelf_index):
							if rb and rb[0] < self.robotParameters.maxShelfDetectionDistance:
								shelfRangeBearing[index] = rb

				# check to see if items are in field of view
				if warehouseObjects.items in objects:
					# Check each item type (0-5: bowl, mug, bottle, soccer, rubiks, cereal) in the scene
					for item_type in range(6):
						if self._is_object_detected(objectsDetected, item_type):
							# Try to find item instances of this type in the simulation
							try:
								# Get all objects in the scene and filter for items of this type
								# This is a simplified approach - in practice you might maintain
								# a list of item handles or use a more efficient detection method
								item_names = ["BOWL", "MUG", "BOTTLE", "SOCCER_BALL", "RUBIKS_CUBE", "CEREAL_BOX"]
								item_name = item_names[item_type]
								
								# Try to get item handle (this may fail if item doesn't exist in scene)
								try:
									item_handle = self.sim.getObject(f'/{item_name}')
									item_position = self.sim.getObjectPosition(item_handle, -1)
									
									# Use the helper function to process this item detection
									result = self._process_single_object_detection(
										item_position, item_type, objectsDetected, 
										self.robotParameters.maxItemDetectionDistance)
									
									if result is not None:
										self._add_item_to_range_bearing(itemRangeBearing, item_type, result)
										
								except Exception:
									# Item of this type may not exist in the scene, or may be at picking stations
									# Check picking stations for items of this type (handled separately)
									pass
									
							except Exception as e:
								# General error handling for item detection
								print(f"Warning: Error processing item type {item_type}: {e}")
				
				# check to see if items at picking stations are in field of view
				if warehouseObjects.items in objects:
					for station_index in range(3):
						item_type = self.sceneParameters.pickingStationContents[station_index]
						
						if item_type != -1 and 0 <= item_type <= 5:  # Valid item type
							station_handle = self.pickingStationHandles[station_index]
							
							if station_handle is not None:
								try:
									# Get the position of the picking station item
									station_position = self.sim.getObjectPosition(station_handle, -1)
									item_position = [station_position[0], station_position[1], station_position[2]]
									
									# Check if item is in camera field of view
									if self._is_object_detected(objectsDetected, item_type) and self.PointInsideArena(item_position):
										_valid, _range, _bearing = self.GetRBInCameraFOV(item_position)
										
										# check range is not too far away
										if _range < self.robotParameters.maxItemDetectionDistance \
											and abs(_bearing) < self.robotParameters.cameraPerspectiveAngle/2:
											# Use helper function to add item detection
											self._add_item_to_range_bearing(itemRangeBearing, item_type, [_range, _bearing])
								except Exception as e:
									# Silently ignore errors for missing picking stations
									pass
				
				# check to see which obstacles are within the field of view
				if warehouseObjects.obstacles in objects:
					for index, obstaclePosition in enumerate(self.obstaclePositions):
						if obstaclePosition != None:
							# Obstacles are at indices 6-8 in the detection array (obstacle_0, obstacle_1, obstacle_2)
							result = self._process_single_object_detection(obstaclePosition, 6 + index, objectsDetected, self.robotParameters.maxObstacleDetectionDistance)
							if result is not None:
								# make obstaclesRangeBearing into empty lists, if currently set to None
								if obstaclesRangeBearing == None:
									obstaclesRangeBearing = []
								obstaclesRangeBearing.append(result)

				# check to see if picking station is in field of view
				if warehouseObjects.pickingStation in objects:
					if self.packingStationPosition != None:
						# Picking station is at index 9 in the detection array
						packingStationRangeBearing = self._process_single_object_detection(
							self.packingStationPosition, 9, objectsDetected, self.robotParameters.maxPackingBayDetectionDistance)

				# check to see if black row markers are in field of view
				if warehouseObjects.row_markers in objects:
					# Row markers are at indices 10-12 in the detection array (row_marker1, row_marker2, row_marker3)
					rowMarkerRangeBearing = self._process_multiple_object_detection(
						self.rowMarkerPositions, 10, objectsDetected, self.robotParameters.maxRowMarkerDetectionDistance)

				# NEW: check to see if individual picking stations (square markers) are in field of view
				if warehouseObjects.PickingStationMarkers in objects:
					for station_index in range(3):
						# Square markers are at indices 19-21 in the detection array (square_marker1-3 = picking stations 1-3)
						if self._is_object_detected(objectsDetected, 19 + station_index):
							# Get picking station position
							station_handle = self.pickingStationHandles[station_index]
							if station_handle is not None:
								try:
									station_position = self.sim.getObjectPosition(station_handle, -1)
									result = self._process_single_object_detection(
										station_position, 19 + station_index, objectsDetected, 
										self.robotParameters.maxPackingBayDetectionDistance)
									if result is not None:
										pickingStationRangeBearing[station_index] = result
								except Exception as e:
									# Silently ignore errors for missing picking stations
									pass

		return itemRangeBearing, packingStationRangeBearing, obstaclesRangeBearing, rowMarkerRangeBearing, shelfRangeBearing, pickingStationRangeBearing


	def GetCameraImage(self):

		if self.cameraHandle == None:
			return None, None
	
		try:
			detectionCount, packet1, packet2 = self.sim.handleVisionSensor(self.cameraHandle)
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
			self.robotParameters.wheelRadius = 0.03

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
				# Instead of calling script function, use ZMQ API directly
				# This replicates the RobotReleaseItem Lua script functionality
				
				# For now, we'll track the held item handle when we collect it
				# This is a simplified version - in the full implementation you'd track multiple items
				if hasattr(self, 'heldItemHandle') and self.heldItemHandle is not None:
					objectHandle = self.heldItemHandle
					
					# Check if handle is valid
					if objectHandle != -1:
						try:
							# Verify handle is still valid
							self.sim.getObjectPosition(objectHandle, -1)
							
							# Unparent the item from the robot (equivalent to sim.setObjectParent(objectHandle, -1, true))
							self.sim.setObjectParent(objectHandle, -1, True)
							
							# Make object respondable (collidable) - equivalent to sim.setObjectInt32Param(objectHandle, 3004, 1)
							# self.sim.setObjectInt32Param(objectHandle, 3004, 1)
							
							# Make object non-static (dynamic) - equivalent to sim.setObjectInt32Param(objectHandle, sim.shapeintparam_static, 0)
							# self.sim.setObjectInt32Param(objectHandle, self.sim.shapeintparam_static, 0)
							
							print("Released item with handle: %d" % objectHandle)
							self.heldItemHandle = None
							self.itemConnectedToRobot = False
							
						except Exception as e:
							print("Error: Invalid item handle during release: %s" % str(e))
							self.heldItemHandle = None
							self.itemConnectedToRobot = False
					else:
						print("Error: Invalid item handle (-1) during release")
						self.heldItemHandle = None
						self.itemConnectedToRobot = False
				else:
					print("Warning: No items to release")
					self.itemConnectedToRobot = False
					
			except Exception as e:
				print("Error during item release: %s" % str(e))
				self.itemConnectedToRobot = False

	# Replicate the JoinRobotAndItem functionality using direct sim calls
	def JoinRobotAndItem(self, item_handle):
		"""
		Replicates the Lua JoinRobotAndItem function using direct ZMQ API calls.
		Sets the item position relative to the robot and parents it.
		
		Args:
			item_handle: Handle of the item to collect
			
		Returns:
			bool: True if successful, False otherwise
		"""
		try:
			# Validate object handle (equivalent to sim.isHandle check in Lua)
			if item_handle == -1:
				print("Warning: Invalid item handle (-1)")
				return False
				
			# Check if handle is valid by trying to get its position
			try:
				self.sim.getObjectPosition(item_handle, -1)
			except Exception:
				print("Warning: Invalid object handle: %s" % str(item_handle))
				return False
			
			# Set object position relative to robot (equivalent to __setObjectPosition__ in Lua)
			# Position [0, 0, 0.1] relative to the robot center
			self.sim.setObjectPosition(item_handle, self.robotHandle, [0.13, 0, -0.06])
			
			# Parent the item to the robot (equivalent to sim.setObjectParent in Lua)
			self.sim.setObjectParent(item_handle, self.robotHandle, True)
			
			# Store the item handle for later release
			self.heldItemHandle = item_handle
			
			print("Grabbed item with handle: %d" % item_handle)
			return True
			
		except Exception as e:
			print("Error in JoinRobotAndItem: %s" % str(e))
			return False

	# Use this to force a physical connection between item and rover
	# Collects items from picking stations only
	# Outputs:
	#		Success - returns True if a item was collected or false if not
	#		closest_station - returns the station number (1-3) if closest_picking_station=True and item collected from picking station
	def CollectItem(self, closest_picking_station=False):

		# If closest_picking_station is True, find the closest item at picking stations
		if closest_picking_station:
			closest_distance = float('inf')
			closest_station = None
			closest_handle = None
			
			for station_index in range(3):
				item_handle = self.pickingStationItemHandles[station_index]
				
				if item_handle is not None:
					try:
						# Get current position of the picking station item
						item_position = self.sim.getObjectPosition(item_handle, self.collectorForceSensorHandle)

						distance = math.sqrt(math.pow(item_position[0], 2) + math.pow(item_position[1], 2) + math.pow(item_position[2], 2))

						if distance is not None and distance < closest_distance:
							closest_distance = distance
							closest_station = station_index
							closest_handle = item_handle
							
					except Exception:
						# Item may have been removed, clear the handle
						self.pickingStationItemHandles[station_index] = None
			
			# Attempt to collect the closest item if within threshold distance
			if closest_station is not None and closest_distance < self.robotParameters.maxCollectDistance:
				if self.itemConnectedToRobot == False:
					# Use our custom JoinRobotAndItem method instead of callScriptFunction
					if self.JoinRobotAndItem(closest_handle):
						self.itemConnectedToRobot = True
						# Clear the item handle since it's been collected
						self.pickingStationItemHandles[closest_station] = None
						print(f"✅ Collected item from picking station {closest_station + 1}! (Distance: {closest_distance:.3f}m)")
						return True, closest_station + 1  # Return success and station number (1-3)
					else:
						print(f"Error joining item from picking station {closest_station + 1}")
						return False, None
				else:
					print("Robot already carrying an item")
					return False, None
			elif closest_station is not None:
				print(f"⚠️ Closest item at picking station {closest_station + 1} is too far away ({closest_distance:.3f}m > {self.robotParameters.maxCollectDistance:.3f}m)")
				return False, None
			else:
				print("⚠️ No items found at any picking stations")
				return False, None

		# Default behavior: try to collect from any picking station within reach
		for station_index in range(3):
			item_handle = self.pickingStationItemHandles[station_index]
			
			if item_handle is not None:
				try:
					# Get current position of the picking station item
					item_position = self.sim.getObjectPosition(item_handle, self.collectorForceSensorHandle)
					distance = math.sqrt(math.pow(item_position[0], 2) + math.pow(item_position[1], 2) + math.pow(item_position[2], 2))
					
					if distance is not None and distance < self.robotParameters.maxCollectDistance and self.itemConnectedToRobot == False:
						# make physical connection between item and robot to simulate collector
						if self.JoinRobotAndItem(item_handle):
							self.itemConnectedToRobot = True
							# Clear the item handle since it's been collected
							self.pickingStationItemHandles[station_index] = None
							print(f"✅ Collected item from picking station {station_index + 1}! (Distance: {distance:.3f}m)")
							return True, station_index + 1
						else:
							print(f"Error joining item from picking station {station_index + 1}")
							return False, None
				except Exception as e:
					# Item may have been removed or doesn't exist anymore
					self.pickingStationItemHandles[station_index] = None
		
		return False, None

	def GetItemBayHeight(self,itemPosition):
		if itemPosition[2] < 0.1:
			return 0
		elif itemPosition[2] < 0.2:
			return 1
		else:
			return 2

	def DropItemInClosestShelfBay(self, max_drop_distance=0.5):
		"""
		Drops an item in the closest shelf bay that is within the threshold distance.
		Uses sim.checkDistance with bay handles to find the closest empty bay.
		
		Args:
			max_drop_distance (float): Maximum distance to consider a shelf bay for dropping (in meters)
			
		Returns:
			tuple: (success, shelf_info) where:
				- success (bool): True if item was dropped successfully, False otherwise
				- shelf_info (dict): Information about the shelf where item was dropped, or None if failed
					Contains: {'shelf': shelf_index, 'x': x_coord, 'y': y_coord, 'distance': distance_to_bay}
		"""
		if not self.itemConnectedToRobot:
			print("❌ No item to drop - robot is not carrying anything")
			return False, None
			
		if self.robotPose is None:
			print("❌ Cannot drop item - robot position unknown")
			return False, None
			
		closest_bay = None
		min_distance = float('inf')
		
		# Check all shelf bays to find the closest empty one using bay handles
		for shelf in range(6):  # 6 shelves (0-5)
			for x in range(4):  # 4 positions along x-axis
				for y in range(3):  # 3 height levels
					# Check if this bay is empty (no item currently there)
					if np.isnan(self.itemPositions[shelf, x, y]).any():
						# Get the bay handle
						bay_handle = self.bayHandles[shelf, x, y]
						
						if bay_handle is not None:
							try:
								# Use sim.checkDistance to get distance from robot to this bay
								# Returns: result, distanceData, objectHandlePair
								# distanceData format: [obj1X, obj1Y, obj1Z, obj2X, obj2Y, obj2Z, dist]
								result, distance_data, object_pair = self.sim.checkDistance(self.collectorForceSensorHandle, bay_handle, max_drop_distance)

								# If distance check succeeded (result == 1 means distance < threshold)
								if result == 1 and distance_data is not None:
									# Extract actual distance from distance_data (7th element)
									if isinstance(distance_data, (list, tuple)) and len(distance_data) >= 7:
										actual_distance = distance_data[6]  # Distance is the 7th element (index 6)
									else:
										# Fallback: if data format is unexpected, skip this bay
										print("Warning: Unexpected distance_data format from sim.checkDistance")
										continue
									
									# Check if this bay is closer than previous candidates
									if actual_distance < min_distance:
										min_distance = actual_distance
										# Get bay position for info
										bay_position = self.sim.getObjectPosition(bay_handle, -1)
										# Debug: Get the actual bay name to verify coordinates
										bay_name = '/Shelf%d/Bay%d%d' % (shelf, x, y)
										closest_bay = {
											'shelf': shelf,
											'x': x, 
											'y': y,
											'distance': actual_distance,
											'world_x': bay_position[0],
											'world_y': bay_position[1],
											'world_z': bay_position[2],
											'handle': bay_handle,
											'bay_name': bay_name  # Add for debugging
										}
							except Exception as e:
								# Bay handle might be invalid or other error
								print("Warning: Error checking distance to bay %d,%d,%d: %s" % (shelf, x, y, str(e)))
								continue
		
		if closest_bay is None:
			print("❌ No empty shelf bay found within %0.1fm of robot" % max_drop_distance)
			return False, None
			
		# Attempt to drop the item
		try:
			print("📦 Dropping item at shelf %d, bay [%d,%d] (%s) (distance: %0.2fm)" % (
				closest_bay['shelf'], closest_bay['x'], closest_bay['y'], closest_bay['bay_name'], closest_bay['distance']))
			
			# Call the existing drop function
			self.Dropitem()
			
			# Update the item position in our tracking array to mark this bay as occupied
			# Note: In a real implementation, you might want to track what specific item was dropped
			self.itemPositions[closest_bay['shelf'], closest_bay['x'], closest_bay['y']] = [
				closest_bay['world_x'], 
				closest_bay['world_y'], 
				closest_bay['world_z']
			]
			
			print("✅ Successfully dropped item in shelf %d, bay [%d,%d] (%s)" % (
				closest_bay['shelf'], closest_bay['x'], closest_bay['y'], closest_bay['bay_name']))
			return True, closest_bay
			
		except Exception as e:
			print("❌ Error dropping item: %s" % str(e))
			return False, None

	# Update Object Positions - call this in every loop of your navigation code (or at the frequency your vision system runs at). 
	# This is required to get correct range and bearings to objects.
	# This function also emulates the collector. The function returns the global pose/position of the robot and the objects too. 
	# However, you should not use these return values in your nagivation code, they are there to help you debug if you wish.
	# returns: 
	#		robotPose - a 6 element array representing the robot's pose (x,y,z,roll,pitch,yaw), or None if was not successfully updated from COPPELIA
	#		itemPosition - a 3 element array representing the item's position (x,y,z), or None if was not successfully updated from COPPELIA
	#		obstaclePositions - a 3 element list, with each index in the list containing a 3 element array representing the item's position (x,y,z), or None if was not successfully updated from COPPELIA
	def UpdateObjectPositions(self):
		# attempt to get object positions from COPPELIA
		self.GetObjectPositions()

		# update item
		self.UpdateItem()

		# return object positions		
		return self.robotPose, self.itemPositions, self.obstaclePositions
	#########################################
	####### COPPELIA API SERVER FUNCTIONS #######
	#########################################
	# These functions are called within the init function

	# Open connection to COPPELIA API Server
	# Open connection to ZMQ Remote API
	def OpenConnectionToZMQ(self, coppelia_server_ip, port=23000):
		print('Attempting connection to CoppeliaSim ZMQ Remote API Server.')
		try:
			# Set a reasonable timeout to prevent hanging
			print(f'Connecting to {coppelia_server_ip}:{port}...')
			self.client = RemoteAPIClient(host=coppelia_server_ip, port=port)
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

		errorCode = self.GetScriptHandle()
		if errorCode != 0:
			print('Failed to get Script handle. Terminating Program. Error Code %d'%(errorCode))
			sys.exit(-1)

		errorCode = self.GetCameraHandle()
		if errorCode != 0:
			print('Failed to get Vision Sensor object handle. Terminating Program. Error Code %d'%(errorCode))
			sys.exit(-1)

		errorCode = self.GetObjectDetectorHandle()
		if errorCode != 0:
			print('Failed to get Object Detector handle. Terminating Program. Error Code %d'%(errorCode))
			sys.exit(-1)

		errorCode = self.GetCollectorForceSensorHandle()
		if errorCode != 0:
			print('Failed to get CollectorForceSensor handle. Terminating Program. Error Code %d'%(errorCode))
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
		
		errorCodes = self.getBayHandles()
		if any([code != 0 for code in errorCodes]):
			print('Failed to get Bay object handles. Terminating Program. Error Codes %s' % str(errorCodes))
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

	# Get Script Handle (attached to Robot object)
	def GetScriptHandle(self):
		try:
			# Try common script paths - the script is usually attached as a child script to the Robot
			self.scriptHandle = self.sim.getObject('/Robot')  # Use robot handle for script calls
			return 0
		except Exception as e:
			print(f"Error getting script handle: {e}")
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

	# Get ZMQ CollectorForceSensor Handle
	def GetCollectorForceSensorHandle(self):
		try:
			self.collectorForceSensorHandle = self.sim.getObject('/Robot/CollectorForceSensor')
			return 0
		except Exception as e:
			print(f"Error getting collector force sensor handle: {e}")
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
			
			# Try to get multiple picking station handles
			for i in range(3):
				try:
					station_name = f'/Picking_station_{i+1}'
					self.pickingStationHandles[i] = self.sim.getObject(station_name)
				except Exception:
					# If specific picking station doesn't exist, keep as None
					self.pickingStationHandles[i] = None
			
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

	# Get ZMQ bay handles for each shelf
	def getBayHandles(self):
		error_codes = []
		for shelf in range(6):  # 6 shelves (0-5)
			for x in range(4):  # 4 x positions (0-3)
				for y in range(3):  # 3 y positions/heights (0-2)
					try:
						# Bay naming convention: /Shelf{shelf}/Bay{x}{y}
						bay_name = '/Shelf%d/Bay%d%d' % (shelf, x, y)
						self.bayHandles[shelf, x, y] = self.sim.getObject(bay_name)
						error_codes.append(0)
					except Exception as e:
						print("Warning: Could not get handle for %s: %s" % (bay_name, str(e)))
						self.bayHandles[shelf, x, y] = None
						error_codes.append(-1)
		return error_codes
	
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
		
		# Set picking station contents
		print('Setting picking station contents...')
		self.SetPickingStationContents()
		
		print('Scene setup completed.')
		
		

	def SetPickingStationContents(self):
		"""Place items at picking stations based on sceneParameters.pickingStationContents"""
		print('Placing items at picking stations...')
		
		for station_index in range(3):
			item_type = self.sceneParameters.pickingStationContents[station_index]
			
			if item_type != -1 and 0 <= item_type <= 5:  # Valid item type
				station_handle = self.pickingStationHandles[station_index]
				
				if station_handle is not None:
					try:
						# Get the position of the picking station
						station_position = self.sim.getObjectPosition(station_handle, -1)
						
						# Place item slightly above the picking station surface
						item_position = [station_position[0], station_position[1], station_position[2]+0.01]
						
						# Copy the item template to this position
						item_names = ["BOWL", "MUG", "BOTTLE", "SOCCER_BALL", "RUBIKS_CUBE", "CEREAL_BOX"]
						template_handle = self.itemTemplateHandles[item_type]
						
						if template_handle is not None:
							# Copy the item from template
							new_item_handle = self.sim.copyPasteObjects([template_handle], 0)[0]
							
							# Store the item handle for collection purposes
							self.pickingStationItemHandles[station_index] = new_item_handle
							
							# Position the item at the picking station
							self.sim.setObjectPosition(new_item_handle, -1, item_position)
							
							print(f'Placed {item_names[item_type]} at picking station {station_index + 1}')
						else:
							print(f'Warning: Template for {item_names[item_type]} not found')
							
					except Exception as e:
						print(f'Error placing item at picking station {station_index + 1}: {e}')
				else:
					print(f'Warning: Picking station {station_index + 1} handle not found')

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
			# Flip the camera horizontally by changing yaw from math.pi/2.0 to -math.pi/2.0
			self.sim.setObjectOrientation(self.cameraHandle, self.robotHandle, [math.pi, pitch, -math.pi/2.0])
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
		# Set camera pose and object position to None so can check in an error occurred
		self.robotPose = None
		self.cameraPose = None
		# self.itemPositions = [None]*len(self.itemHandles)
		self.packingStationPosition = None
		self.obstaclePositions = [None, None, None]

		# GET 2D ROBOT POSE
		try:
			robotPosition = self.sim.getObjectPosition(self.robotHandle, -1)
			robotOrientation = self.sim.getObjectOrientation(self.robotHandle, -1)
			self.robotPose = [robotPosition[0], robotPosition[1], robotPosition[1], robotOrientation[0], robotOrientation[1], robotOrientation[2]]
		except Exception as e:
			print(f"Error getting robot pose: {e}")

		# GET 3D CAMERA POSE
		try:
			cameraPosition = self.sim.getObjectPosition(self.cameraHandle, -1)
			self.cameraPose = [cameraPosition[0], cameraPosition[1], cameraPosition[2], robotOrientation[0], robotOrientation[1], robotOrientation[2]]
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
			packingStationPosition = self.sim.getObjectPosition(self.packingStationHandle, -1)
			self.packingStationPosition = packingStationPosition
		except Exception as e:
			print(f"Error getting picking station position: {e}")

		# obstacle positions
		obstaclePositions = [None, None, None]
		for index, obs in enumerate(self.obstaclePositions):
			try:
				obstaclePositions[index] = self.sim.getObjectPosition(self.obstacleHandles[index], -1)
				self.obstaclePositions[index] = obstaclePositions[index]
			except Exception as e:
				print(f"Error getting obstacle position {index}: {e}")

		# row marker positions
		rowMarkerPositions = [None,None,None]
		for index, rowMarker in enumerate(self.rowMarkerPositions):
			try:
				rowMarkerPositions[index] = self.sim.getObjectPosition(self.rowMarkerHandles[index], -1)
				self.rowMarkerPositions[index] = rowMarkerPositions[index]
			except Exception as e:
				print(f"Error getting row marker position {index}: {e}")

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


				if self.itemConnectedToRobot == True:
					# random chance to disconnect
					if np.random.rand() > self.robotParameters.collectorQuality:
						# terminate connection between item and robot to simulate collector
						try:
							self.sim.callScriptFunction('RobotReleaseItem', self.scriptHandle, [], [], [], "")
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

	# Gets the range and bearing to all shelves from the camera position
	def GetShelfRangeBearing(self):
		"""
		Calculate range and bearing to all shelves from the camera position.
		
		Returns:
			list: A list of [range, bearing] pairs for each shelf (6 shelves total).
				  Returns None for shelves that cannot be detected or don't exist.
		"""
		shelfRB = [None] * 6  # Initialize list for 6 shelves
		
		if self.cameraPose is None:
			return shelfRB
			
		cameraPose2D = [self.cameraPose[0], self.cameraPose[1], self.cameraPose[5]]
		
		for shelf_index in range(6):
			shelf_handle = self.shelfHandles[shelf_index]
			
			if shelf_handle is not None:
				try:
					# Get the shelf position
					shelf_position = self.sim.getObjectPosition(shelf_handle, -1)
					
					# Calculate range and bearing from camera to shelf
					_range, _bearing = self.GetRangeAndBearingFromPoseAndPoint(cameraPose2D, shelf_position)
					
					# Check if shelf is within detection range and field of view
					if _range < self.robotParameters.maxShelfDetectionDistance:
						# Check if the shelf is within the camera's field of view
						if abs(_bearing) < self.robotParameters.cameraPerspectiveAngle / 2:
							shelfRB[shelf_index] = [_range, _bearing]
				except Exception as e:
					# If we can't get the shelf position, leave it as None
					continue
		
		return shelfRB


def print_debug_range_bearing(object_type, range_bearing_data):
	"""Debug function to print range and bearing information for detected objects
	
	Args:
		object_type (str): Name/description of the object type being displayed
		range_bearing_data: Range and bearing data (can be single [range, bearing] or list of such pairs)
	"""
	if range_bearing_data is None:
		print(f"🔍 {object_type}: No objects detected")
		return
	
	# Special handling for items array (6-element array with one element per item type)
	if object_type == "Items" and isinstance(range_bearing_data, list) and len(range_bearing_data) == 6:
		item_names = ["Bowls", "Mugs", "Bottles", "Soccer Balls", "Rubiks Cubes", "Cereal Boxes"]
		any_items_found = False
		
		for item_type, detections in enumerate(range_bearing_data):
			if detections is not None and len(detections) > 0:
				any_items_found = True
				for i, rb in enumerate(detections):
					if rb is not None and len(rb) >= 2:
						range_m = rb[0]
						bearing_rad = rb[1]
						bearing_deg = math.degrees(bearing_rad)
						print(f"🔍 {item_names[item_type]}[{i}]: Range = {range_m:.3f}m, Bearing = {bearing_rad:.3f}rad ({bearing_deg:.1f}°)")
		
		if not any_items_found:
			print(f"🔍 {object_type}: No items detected")
		return
	
	if isinstance(range_bearing_data, list):
		if len(range_bearing_data) == 0:
			print(f"🔍 {object_type}: Empty list (no detections)")
			return
		
		# Check if this is a single [range, bearing] pair or a list of pairs
		if len(range_bearing_data) == 2 and isinstance(range_bearing_data[0], (int, float)) and isinstance(range_bearing_data[1], (int, float)):
			# Single detection with range and bearing
			range_m = range_bearing_data[0]
			bearing_rad = range_bearing_data[1]
			bearing_deg = math.degrees(bearing_rad)
			print(f"🔍 {object_type}: Range = {range_m:.3f}m, Bearing = {bearing_rad:.3f}rad ({bearing_deg:.1f}°)")
		else:
			# List of multiple detections
			for i, rb in enumerate(range_bearing_data):
				if rb is not None and isinstance(rb, list) and len(rb) >= 2:
					# Check if this is a nested list like [[range, bearing]]
					if isinstance(rb[0], list) and len(rb[0]) >= 2:
						# Handle nested structure [[range, bearing]]
						inner_rb = rb[0]
						range_m = inner_rb[0]
						bearing_rad = inner_rb[1]
						bearing_deg = math.degrees(bearing_rad)
						print(f"🔍 {object_type}[{i}]: Range = {range_m:.3f}m, Bearing = {bearing_rad:.3f}rad ({bearing_deg:.1f}°)")
					else:
						# Normal [range, bearing] structure
						range_m = rb[0]
						bearing_rad = rb[1]
						bearing_deg = math.degrees(bearing_rad)
						print(f"🔍 {object_type}[{i}]: Range = {range_m:.3f}m, Bearing = {bearing_rad:.3f}rad ({bearing_deg:.1f}°)")
				elif rb is None:
					print(f"🔍 {object_type}[{i}]: None (not detected)")
				else:
					print(f"🔍 {object_type}[{i}]: Invalid data - {rb}")
	else:
		print(f"🔍 {object_type}: Invalid format - {range_bearing_data}")

# Parameter classes for robot and scene configuration
class RobotParameters(object):
	"""Parameters for configuring the warehouse robot"""
	def __init__(self):
		# Drive Parameters
		self.driveType = 'differential'  # currently only 'differential' implemented
		self.minimumLinearSpeed = 0.0   # minimum speed in m/s
		self.maximumLinearSpeed = 0.25  # maximum speed in m/s
		self.driveSystemQuality = 1.0   # quality from 0 to 1 (1 = perfect)
		
		# Wheel Parameters (set automatically for differential drive)
		self.wheelBase = 0.15           # distance between wheels in m
		self.wheelRadius = 0.03         # wheel radius in m
		
		# Camera Parameters
		self.cameraOrientation = 'landscape'  # 'landscape' or 'portrait'
		self.cameraDistanceFromRobotCenter = 0.1  # distance from robot center in m
		self.cameraHeightFromFloor = 0.15     # height from floor in m
		self.cameraTilt = 0.0                 # tilt angle in radians
		self.cameraResolutionX = 640          # camera width in pixels
		self.cameraResolutionY = 480          # camera height in pixels
		self.cameraPerspectiveAngle = math.pi/3  # field of view angle in radians
		
		# Detection Parameters
		self.maxItemDetectionDistance = 1.0      # max distance to detect items in m
		self.maxPackingBayDetectionDistance = 2.5  # max distance to detect packing bay in m
		self.maxObstacleDetectionDistance = 1.5  # max distance to detect obstacles in m
		self.maxRowMarkerDetectionDistance = 2.5  # max distance to detect row markers in m
		self.maxShelfDetectionDistance = 2.0     # max distance to detect shelves in m
		
		# Collector Parameters
		self.collectorQuality = 1.0      # collector quality from 0 to 1
		self.maxCollectDistance = 0.15   # max distance for collection in m
		
		# Simulation Parameters
		self.sync = False  # synchronous mode (deprecated with ZMQ Remote API)


class SceneParameters(object):
	"""Parameters for configuring the warehouse scene"""
	def __init__(self):
		# Picking station contents [station]. Set to -1 to leave empty.
		# Index 0 = picking station 1, Index 1 = picking station 2, Index 2 = picking station 3
		self.pickingStationContents = [-1, -1, -1]
		
		# Bay contents [shelf][x][y]. Set to -1 for empty bays.
		# shelf: 0-5, x: 0-3, y: 0-2 (height levels)
		# Not used currently
		self.bayContents = np.full((6, 4, 3), -1, dtype=int)
		
		# Obstacle starting positions [x, y] in metres
		# Set to -1 to use current CoppeliaSim position, None if not wanted in scene
		self.obstacle0_StartingPosition = None
		self.obstacle1_StartingPosition = None  
		self.obstacle2_StartingPosition = None
		
		# Robot starting position [x, y, theta] in metres and radians
		# Set to None to use current CoppeliaSim position
		self.robotStartingPosition = None

