#!/usr/bin/python

# import the warehouse bot module - this will include math, time, numpy (as np) and CoppeliaSim ZMQ Remote API modules
from warehousebot_lib import *

#import any other required python modules


# SET SCENE PARAMETERS
sceneParameters = SceneParameters()

# Starting contents of picking stations [station]. Set to -1 to leave empty.
# Index 0 = picking station 1, Index 1 = picking station 2, Index 2 = picking station 3
sceneParameters.pickingStationContents[0] = warehouseObjects.bowl    # Bowl at picking station 1
sceneParameters.pickingStationContents[1] = warehouseObjects.mug     # Mug at picking station 2
sceneParameters.pickingStationContents[2] = warehouseObjects.bottle  # Bottle at picking station 3

# Starting positions of obstacles [x, y] (in metres), -1 to use current CoppeliaSim position, None if not wanted in scene
sceneParameters.obstacle0_StartingPosition = -1  # Use current position
sceneParameters.obstacle1_StartingPosition = -1  # Use current position
sceneParameters.obstacle2_StartingPosition = -1  # Use current position


# SET ROBOT PARAMETERS
robotParameters = RobotParameters()

# Drive Parameters
robotParameters.driveType = 'differential'	# specify if using differential or omni drive system
robotParameters.minimumLinearSpeed = 0.0  	# minimum speed at which your robot can move forward in m/s
robotParameters.maximumLinearSpeed = 0.25 	# maximum speed at which your robot can move forward in m/s
robotParameters.driveSystemQuality = 1		# specifies how good your drive system is from 0 to 1 (with 1 being able to drive in a perfectly straight line when told to do so)

# Camera Parameters
robotParameters.cameraOrientation = 'landscape' # specifies the orientation of the camera, either landscape or portrait
robotParameters.cameraDistanceFromRobotCenter = 0.1 # distance between the camera and the center of the robot in the direction of the collector in metres
robotParameters.cameraHeightFromFloor = 0.15 # height of the camera relative to the floor in metres
robotParameters.cameraTilt = 0.0 # tilt of the camera in radians

# Vision Processing Parameters
robotParameters.maxItemDetectionDistance = 1 # the maximum distance away that you can detect the items in metres
robotParameters.maxPackingBayDetectionDistance = 2.5 # the maximum distance away that you can detect the picking station in metres
robotParameters.maxObstacleDetectionDistance = 1.5 # the maximum distance away that you can detect the obstacles in metres
robotParameters.maxRowMarkerDetectionDistance = 2.5 # the maximum distance away that you can detect the row markers in metres

# Collector Parameters
robotParameters.collectorQuality = 1 # specifies how good your item collector is from 0 to 1.0 (with 1.0 being awesome and 0 being non-existent)
robotParameters.maxCollectDistance = 0.15 #specificies the operating distance of the automatic collector function. Item needs to be less than this distance to the collector

robotParameters.sync = False # This parameter forces the simulation into synchronous mode when True; requiring you to call stepSim() to manually step the simulator in your loop (Note: sync mode is deprecated with ZMQ Remote API)


# MAIN SCRIPT
if __name__ == '__main__':

	# Wrap everything in a try except case that catches KeyboardInterrupts. 
	# In the exception catch code attempt to Stop CoppeliaSim so you don't have to Stop it manually when pressing CTRL+C
	try:
		print("🤖 Starting EGB320 CoppeliaSim Warehouse Robot Example")
		print("Press Ctrl+C to stop the simulation\n")

		# Create CoppeliaSim WarehouseBot object - this will attempt to open a connection to CoppeliaSim using ZMQ Remote API. 
		# Make sure CoppeliaSim is running with ZMQ Remote API enabled (default in modern versions).
		# Check the port number in the CoppeliaSim settings if you have issues connecting to ZMQ Remote API
		warehouseBotSim = COPPELIA_WarehouseRobot(robotParameters, sceneParameters, coppelia_server_ip='127.0.0.1', port=23000)
		warehouseBotSim.StartSimulator()

		# Main control loop
		# Consider changing this to a rate control loop using time.sleep() to avoid busy waiting
		# for example, you could use time.sleep(0.1) to run at 10Hz
		while True:
			
			# Set robot velocities (forward velocity, turn velocity)
			warehouseBotSim.SetTargetVelocities(0.2, 1.0)

			# Get detected objects in the camera's field of view
			# returns are Range and Bearing for each object type
			objectsRB = warehouseBotSim.GetDetectedObjects([
				warehouseObjects.items,
				warehouseObjects.shelves,
				warehouseObjects.row_markers,
				warehouseObjects.obstacles,
				warehouseObjects.pickingStation,
			])

			# Unpack the detected objects
			itemsRB, packingStationRB, obstaclesRB, rowMarkerRangeBearing, shelfRangeBearing = objectsRB

			# Read proximity sensor
			#dist = warehouseBotSim.readProximity()
			
			# Update object positions (required for accurate range and bearing calculations)
			warehouseBotSim.UpdateObjectPositions()
			
			# Check for items in field of view and attempt to collect them
			#warehouseBotSim.CollectItem()

			# Check for close obstacles and alert
			if obstaclesRB is not None:
				for obstacle in obstaclesRB:
					obstacleRange = obstacle[0]
					obstacleBearing = obstacle[1]
					
					# Alert if obstacle is close (within 1 meter)
					if obstacleRange < 1.0:
						print(f"⚠️  Close obstacle! Range: {obstacleRange:.3f}m, Bearing: {obstacleBearing:.3f}rad")

			# For synchronous mode (not currently implemented with ZMQ Remote API)
			#if robotParameters.sync:
			#	warehouseBotSim.stepSim()

	except KeyboardInterrupt:
		print("\n🛑 Keyboard interrupt detected - stopping simulation...")
		warehouseBotSim.StopSimulator()
		print("✅ Simulation stopped successfully")



