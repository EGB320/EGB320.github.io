#!/usr/bin/python


# import the warehouse bot module - this will include math, time, numpy (as np) and CoppeliaSim ZMQ Remote API modules
from warehousebot_lib import *
import cv2
import pygame

#import any other required python modules


# SET SCENE PARAMETERS
sceneParameters = SceneParameters()

# Starting contents of the bays [shelf,X,Y]. Set to -1 to leave empty.
sceneParameters.bayContents = np.random.random_integers(0,5,sceneParameters.bayContents.shape)
sceneParameters.bayContents[0,3,1] = warehouseObjects.bowl
sceneParameters.bayContents[1,1,2] = warehouseObjects.mug
sceneParameters.bayContents[2,3,1] = warehouseObjects.bottle

# Starting contents of picking stations [station]. Set to -1 to leave empty.
# Index 0 = picking station 1, Index 1 = picking station 2, Index 2 = picking station 3
sceneParameters.pickingStationContents[0] = warehouseObjects.bowl    # Bowl at picking station 1
sceneParameters.pickingStationContents[1] = warehouseObjects.mug     # Mug at picking station 2
sceneParameters.pickingStationContents[2] = warehouseObjects.bottle  # Bottle at picking station 3

# sceneParameters.obstacle0_StartingPosition = [-0.45, 0.5]  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current CoppeliaSim position, or none if not wanted in the scene
sceneParameters.obstacle0_StartingPosition = None  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current CoppeliaSim position, or none if not wanted in the scene
sceneParameters.obstacle1_StartingPosition = None   # starting position of obstacle 1 [x, y] (in metres), -1 if want to use current CoppeliaSim position, or none if not wanted in the scene
sceneParameters.obstacle2_StartingPosition = None   # starting position of obstacle 2 [x, y] (in metres), -1 if want to use current CoppeliaSim position, or none if not wanted in the scene

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
robotParameters.cameraResolutionX = 320 # camera resolution width in pixels
robotParameters.cameraResolutionY = 240 # camera resolution height in pixels

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
	# In the exception catch code attempt to Stop the CoppeliaSim Simulator so don't have to Stop it manually when pressing CTRL+C
	try:

		print("🤖 Starting EGB320 FPV Keyboard Example with ZMQ Remote API")
		print("📹 Camera stream will display in pygame window")
		print("Press Ctrl+C to stop the simulation\n")

		pygame.init()
		pygame.display.set_caption("CoppeliaSim Camera Stream")
		done = False

		# Create CoppeliaSim WarehouseBot object - this will attempt to open a connection to CoppeliaSim using ZMQ Remote API
		# Make sure CoppeliaSim is running with ZMQ Remote API enabled (default in modern versions)
		warehouseBotSim = COPPELIA_WarehouseRobot(robotParameters, sceneParameters, coppelia_server_ip='127.0.0.1', port=23000)
		warehouseBotSim.StartSimulator()

		print("Getting initial camera image...")
		image = None
		while image == None:
			resolution, image = warehouseBotSim.GetCameraImage()
			
		print(f"Camera resolution: {resolution}")
		screen = pygame.display.set_mode(resolution)
		
		#We recommend changing this to a controlled rate loop (fixed frequency) to get more reliable control behaviour
		print("Starting main loop - use arrow keys or WASD to control robot")
		print("Controls:")
		print("  W/↑ : Move forward")
		print("  S/↓ : Move backward") 
		print("  A/← : Turn left")
		print("  D/→ : Turn right")
		print("  Space: Pickup item")
		print("  R: Drop item in closest shelf bay")
		print("  P: Stop/Pause movement")
		print("Close pygame window or press Ctrl+C to exit\n")
		
		# Robot control variables
		forward_speed = 0.0
		turn_speed = 0.0
		max_speed = 0.2
		max_turn = 3.0
		
		clock = pygame.time.Clock()
		
		while not done:
			# Handle keyboard input for robot control
			keys = pygame.key.get_pressed()
			
			# Reset speeds
			forward_speed = 0.0
			turn_speed = 0.0
			
			# Forward/backward control
			if keys[pygame.K_w] or keys[pygame.K_UP]:
				forward_speed = max_speed
			elif keys[pygame.K_s] or keys[pygame.K_DOWN]:
				forward_speed = -max_speed
				
			# Turn control
			if keys[pygame.K_a] or keys[pygame.K_LEFT]:
				turn_speed = max_turn
			elif keys[pygame.K_d] or keys[pygame.K_RIGHT]:
				turn_speed = -max_turn
				
			# Stop command
			if keys[pygame.K_p]:
				forward_speed = 0.0
				turn_speed = 0.0
			
			# Send control commands to robot
			warehouseBotSim.SetTargetVelocities(forward_speed, turn_speed)

			# Get camera image for display
			resolution, image = warehouseBotSim.GetCameraImage()

			if image is not None:
				try:
					# Validate image data before processing
					if isinstance(image, (list, tuple)) and len(image) > 0:
						cv2_image = np.array(image, dtype=np.uint8)
						
						# Check if we have valid image dimensions
						expected_pixels = resolution[0] * resolution[1] * 3  # RGB
						if len(cv2_image) == expected_pixels:
							cv2_image = cv2_image.reshape([resolution[1], resolution[0], 3])

							screen.fill([0, 0, 0])
							# Camera orientation corrected in sim, but still need to flip image for display
							frame = cv2.flip(cv2_image, 0)  # Flip vertically (upside down) for correct display
							frame = frame.swapaxes(0, 1)
							frame = pygame.surfarray.make_surface(frame)
							screen.blit(frame, (0, 0))
							
							# Display control info on screen
							font = pygame.font.Font(None, 24)
							speed_text = font.render(f"Speed: {forward_speed:.2f} m/s", True, (255, 255, 255))
							turn_text = font.render(f"Turn: {turn_speed:.2f} rad/s", True, (255, 255, 255))
							controls_text = font.render("Space: Pickup | R: Drop | P: Stop", True, (255, 255, 0))
							screen.blit(speed_text, (10, 10))
							screen.blit(turn_text, (10, 35))
							screen.blit(controls_text, (10, 60))
							
							pygame.display.update()
						else:
							print(f"Warning: Image size mismatch. Expected {expected_pixels}, got {len(cv2_image)}")
					else:
						print("Warning: Invalid image data received")
				except Exception as e:
					print(f"Error processing camera image: {e}")
					# Display a black screen with error message
					screen.fill([0, 0, 0])
					font = pygame.font.Font(None, 36)
					error_text = font.render("Camera Error - Check CoppeliaSim Vision Sensor", True, (255, 0, 0))
					screen.blit(error_text, (10, 50))
					pygame.display.update()

			# Handle pygame events
			for event in pygame.event.get():
				if event.type == pygame.QUIT:
					done = True
				elif event.type == pygame.KEYDOWN:
					if event.key == pygame.K_ESCAPE:
						done = True
					elif event.key == pygame.K_SPACE:
						# Pickup item functionality
						try:
							# Use the enhanced CollectItem function with closest_picking_station=True
							# This will automatically find the closest item at picking stations and collect it if within range
							result, station_num = warehouseBotSim.CollectItem(closest_picking_station=True)
							
							if not result:
								print("❌ No items collected")
								
						except Exception as e:
							print(f"Error during pickup attempt: {e}")
					elif event.key == pygame.K_r:
						# Drop item functionality
						try:
							# Drop item in the closest shelf bay within 0.5m
							result, shelf_info = warehouseBotSim.DropItemInClosestShelfBay(max_drop_distance=0.1)
							
							if result:
								print("✅ Item dropped successfully at shelf %d, bay [%d,%d]" % (
									shelf_info['shelf'], shelf_info['x'], shelf_info['y']))
							else:
								print("❌ Could not drop item - no suitable shelf bay found or not carrying item")
								
						except Exception as e:
							print("Error during drop attempt: %s" % str(e))

			# Optional: Get detected objects for debugging (uncomment if needed)
			# objectsRB = warehouseBotSim.GetDetectedObjects()
			# itemsRB, packingStationRB, obstaclesRB, rowMarkerRangeBearing, shelfRangeBearing = objectsRB

			# Update object positions (less frequently for better performance)
			# Only update every 10th frame to reduce ZMQ API overhead
			if pygame.time.get_ticks() % 100 < 17:  # ~10 times per second
				warehouseBotSim.UpdateObjectPositions()
			
			# Limit frame rate to 60 FPS
			clock.tick(60)

	except KeyboardInterrupt as e:
		print("\n🛑 Keyboard interrupt detected - stopping simulation...")
		# attempt to stop simulator so it restarts and don't have to manually press the Stop button in CoppeliaSim 
		warehouseBotSim.StopSimulator()
		print("✅ Simulation stopped successfully")
		pygame.quit()



