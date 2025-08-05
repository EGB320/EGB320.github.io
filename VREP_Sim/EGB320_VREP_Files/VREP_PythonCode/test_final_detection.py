#!/usr/bin/env python3

"""
Final test script for object detection functionality
"""

from warehousebot_lib import *
import time

# Robot Parameters
robotParameters = RobotParameters()
robotParameters.driveType = 'differential'
robotParameters.sync = False

# Scene Parameters  
sceneParameters = SceneParameters()

print("Testing Object Detection with Direct Vision Sensor...")
try:
    # Create robot object
    robot = COPPELIA_WarehouseRobot('127.0.0.1', robotParameters, sceneParameters)
    
    print("Starting simulator...")
    robot.StartSimulator()
    
    # Wait for simulation to settle
    time.sleep(2)
    
    print("\n=== Testing Object Detection ===")
    for i in range(3):
        print(f"\nDetection cycle {i+1}:")
        try:
            # Test object detection - focusing only on objects that should work
            items, packing_bay, obstacles, row_markers, shelves = robot.GetDetectedObjects(
                [warehouseObjects.packingBay, warehouseObjects.obstacles]
            )
            
            print(f"  Packing bay detected: {packing_bay}")
            print(f"  Obstacles detected: {obstacles}")
            
            if packing_bay:
                print(f"    - Range: {packing_bay[0]:.3f}m, Bearing: {packing_bay[1]:.3f} rad")
            
            if obstacles:
                for j, obs in enumerate(obstacles):
                    print(f"    - Obstacle {j}: Range {obs[0]:.3f}m, Bearing {obs[1]:.3f} rad")
                    
        except Exception as e:
            print(f"  Error in detection cycle {i+1}: {e}")
        
        time.sleep(1)
    
    print("\nStopping simulator...")
    robot.StopSimulator()
    print("Test completed successfully!")
    
except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()
