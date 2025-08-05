#!/usr/bin/python


# import the warehouse bot module - this will include math, time, numpy (as np) and CoppeliaSim ZMQ Remote API modules
from warehousebot_lib import *
import time
from collections import defaultdict
#import any other required python modules

# Timing tracking variables
timing_stats = defaultdict(list)
benchmark_enabled = True

def time_function(func_name):
    """Decorator to time function calls"""
    def decorator(func):
        def wrapper(*args, **kwargs):
            if benchmark_enabled:
                start_time = time.perf_counter()
                result = func(*args, **kwargs)
                end_time = time.perf_counter()
                timing_stats[func_name].append((end_time - start_time) * 1000)  # Convert to milliseconds
                return result
            else:
                return func(*args, **kwargs)
        return wrapper
    return decorator

def print_timing_stats(loop_count):
    """Print timing statistics every N loops"""
    if not timing_stats:
        return
        
    print(f"\n📊 === Timing Statistics (after {loop_count} loops) ===")
    total_time = 0
    
    for func_name, times in timing_stats.items():
        if times:
            avg_time = sum(times) / len(times)
            min_time = min(times)
            max_time = max(times)
            total_time += avg_time
            
            print(f"  {func_name}:")
            print(f"    Avg: {avg_time:.2f}ms, Min: {min_time:.2f}ms, Max: {max_time:.2f}ms, Calls: {len(times)}")
    
    print(f"  Total average loop time: {total_time:.2f}ms ({1000/total_time:.1f} FPS)")
    print(f"  Most expensive operation: {max(timing_stats.keys(), key=lambda k: sum(timing_stats[k])/len(timing_stats[k]) if timing_stats[k] else 0)}")
    
    # Clear stats for next period
    timing_stats.clear()


# SET SCENE PARAMETERS
sceneParameters = SceneParameters()

# Starting contents of the bays [shelf,X,Y]. Set to -1 to leave empty.
sceneParameters.bayContents = np.random.random_integers(0,5,sceneParameters.bayContents.shape)
sceneParameters.bayContents[0,3,1] = warehouseObjects.bowl
sceneParameters.bayContents[1,1,2] = warehouseObjects.mug
sceneParameters.bayContents[2,3,1] = warehouseObjects.bottle
sceneParameters.bayContents[3,1,2] = warehouseObjects.soccer
sceneParameters.bayContents[4,2,0] = warehouseObjects.rubiks
sceneParameters.bayContents[5,0,1] = warehouseObjects.cereal


sceneParameters.obstacle0_StartingPosition = [-0.5,0]  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current CoppeliaSim position, or none if not wanted in the scene
# sceneParameters.obstacle0_StartingPosition = None  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current CoppeliaSim position, or none if not wanted in the scene
sceneParameters.obstacle1_StartingPosition = -1   # starting position of obstacle 1 [x, y] (in metres), -1 if want to use current CoppeliaSim position, or none if not wanted in the scene
sceneParameters.obstacle2_StartingPosition = -1   # starting position of obstacle 2 [x, y] (in metres), -1 if want to use current CoppeliaSim position, or none if not wanted in the scene


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
robotParameters.maxPackingBayDetectionDistance = 2.5 # the maximum distance away that you can detect the packing bay in metres
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
		print("📊 Debug info and timing benchmarks enabled")
		print("🔍 Item detection and close obstacle alerts enabled")
		print("⏱️  Performance statistics will be displayed every 100 loops")
		print("Press Ctrl+C to stop the simulation\n")

		# Create CoppeliaSim WarehouseBot object - this will attempt to open a connection to CoppeliaSim using ZMQ Remote API. 
		# Make sure CoppeliaSim is running with ZMQ Remote API enabled (default in modern versions).
		warehouseBotSim = COPPELIA_WarehouseRobot('127.0.0.1', robotParameters, sceneParameters)
		# warehouseBotSim.SetScene()
		
		# Enable detailed timing debug inside functions
		warehouseBotSim.enable_timing_debug = True
		
		warehouseBotSim.StartSimulator()

		#We recommend changing this to a controlled rate loop (fixed frequency) to get more reliable control behaviour
		loop_count = 0

		while True:
			loop_count += 1
			loop_start_time = time.perf_counter()
			
			# Time the motor velocity command
			start_time = time.perf_counter()
			warehouseBotSim.SetTargetVelocities(0.00, 0.3)
			timing_stats["SetTargetVelocities"].append((time.perf_counter() - start_time) * 1000)

			# Time the object detection
			start_time = time.perf_counter()
			objectsRB = warehouseBotSim.GetDetectedObjects(
				[
					warehouseObjects.items,
     				warehouseObjects.shelves,
					warehouseObjects.row_markers,
					warehouseObjects.obstacles,
					warehouseObjects.packingBay,
				]
			)
			timing_stats["GetDetectedObjects"].append((time.perf_counter() - start_time) * 1000)
			itemsRB, packingBayRB, obstaclesRB, rowMarkerRangeBearing, shelfRangeBearing = objectsRB

			# Time the proximity sensor reading
			start_time = time.perf_counter()
			dist = warehouseBotSim.readProximity()
			timing_stats["readProximity"].append((time.perf_counter() - start_time) * 1000)
			
			# Time the UpdateObjectPositions call
			start_time = time.perf_counter()
			warehouseBotSim.UpdateObjectPositions()
			timing_stats["UpdateObjectPositions"].append((time.perf_counter() - start_time) * 1000)
			
			loop_total_time = (time.perf_counter() - loop_start_time) * 1000
			timing_stats["Total Loop Time"].append(loop_total_time)
			
			# Debug info - show every 50 loops, timing stats every 100 loops
			if loop_count % 50 == 0:
				print(f"\n=== Debug Info (Loop {loop_count}) ===")
				print(f"Proximity sensor: {dist:.3f}m")
				
				# Count detected objects
				item_count = sum(1 for item_class in itemsRB if item_class is not None for item in item_class)
				obstacle_count = len(obstaclesRB) if obstaclesRB is not None else 0
				shelf_count = sum(1 for shelf in shelfRangeBearing if shelf is not None)
				row_marker_count = sum(1 for marker in rowMarkerRangeBearing if marker is not None)
				
				print(f"Objects detected:")
				print(f"  - Items: {item_count}")
				print(f"  - Packing bay: {'Yes' if packingBayRB is not None else 'No'}")
				print(f"  - Obstacles: {obstacle_count}")
				print(f"  - Shelves: {shelf_count}/6")
				print(f"  - Row markers: {row_marker_count}/3")
				
				# Show packing bay details if detected
				if packingBayRB is not None:
					print(f"  - Packing bay range: {packingBayRB[0]:.3f}m, bearing: {packingBayRB[1]:.3f}rad")
				
				# Show closest obstacle if any
				if obstaclesRB is not None and len(obstaclesRB) > 0:
					closest_obstacle = min(obstaclesRB, key=lambda x: x[0])
					print(f"  - Closest obstacle: {closest_obstacle[0]:.3f}m, bearing: {closest_obstacle[1]:.3f}rad")
				
				# Show row marker details if detected
				for i, marker in enumerate(rowMarkerRangeBearing):
					if marker is not None:
						print(f"  - Row marker {i+1}: {marker[0]:.3f}m, bearing: {marker[1]:.3f}rad")
				
				# Show current loop performance
				if timing_stats["Total Loop Time"]:
					recent_avg = sum(timing_stats["Total Loop Time"][-10:]) / min(10, len(timing_stats["Total Loop Time"]))
					print(f"  - Current loop time: {recent_avg:.1f}ms ({1000/recent_avg:.1f} FPS)")
			
			# Print detailed timing statistics every 100 loops
			if loop_count % 100 == 0:
				print_timing_stats(loop_count)

			#Check to see if an item is within the camera's FOV
			for itemClass in itemsRB:
				if itemClass != None:
					# loop through each item detected using Pythonian way
					for itemRB in itemClass:
						itemRange = itemRB[0]
						itemBearing = itemRB[1]
						
						print(f"Item detected! Range: {itemRange:.3f}m, Bearing: {itemBearing:.3f}rad")
						
						# Time the collect item operation
						start_time = time.perf_counter()
						warehouseBotSim.CollectItem(1)
						timing_stats["CollectItem"].append((time.perf_counter() - start_time) * 1000)

			# warehouseBotSim.Dropitem()

			# Check to see if any obstacles are within the camera's FOV
			if obstaclesRB != None:
				# loop through each obstacle detected using Pythonian way
				for obstacle in obstaclesRB:
					obstacleRange = obstacle[0]
					obstacleBearing = obstacle[1]
					
					# Alert if obstacle is close (within 1 meter)
					if obstacleRange < 1.0:
						print(f"⚠️  Obstacle detected! Range: {obstacleRange:.3f}m, Bearing: {obstacleBearing:.3f}rad")

			if robotParameters.sync:
				warehouseBotSim.stepSim()  # Note: stepSim() is deprecated with ZMQ Remote API


	except KeyboardInterrupt as e:
		print("\n🛑 Keyboard interrupt detected - stopping simulation...")
		
		# Print final timing statistics
		if timing_stats:
			print("\n📊 === Final Performance Summary ===")
			for func_name, times in timing_stats.items():
				if times:
					avg_time = sum(times) / len(times)
					print(f"  {func_name}: {avg_time:.2f}ms average ({len(times)} calls)")
		
		# attempt to stop simulator so it restarts and you don't have to manually press the Stop button in CoppeliaSim 
		warehouseBotSim.StopSimulator()
		print("✅ Simulation stopped successfully")



