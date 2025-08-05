#!/usr/bin/env python3

"""
Test script for vision sensor data structure debugging
"""

from warehousebot_lib import *
import time

# Robot Parameters
robotParameters = RobotParameters()
robotParameters.driveType = 'differential'
robotParameters.sync = False

# Scene Parameters
sceneParameters = SceneParameters()

print("Connecting to CoppeliaSim...")
try:
    # Create robot object
    robot = COPPELIA_WarehouseRobot('127.0.0.1', robotParameters, sceneParameters)
    
    print("Starting simulator...")
    robot.StartSimulator()
    
    print("Testing vision sensor...")
    
    # Wait a moment for everything to settle
    time.sleep(2)
    
    # Test the vision sensor directly
    try:
        print("\n=== Direct Vision Sensor Test ===")
        result, data, _ = robot.sim.handleVisionSensor(robot.objectDetectorHandle)
        print(f"Result: {result}")
        print(f"Data: {data}")
        print(f"Data type: {type(data)}")
        
        if data and len(data) > 0:
            print(f"Data length: {len(data)}")
            for i, item in enumerate(data):
                print(f"Data[{i}]: {item} (type: {type(item)})")
        
    except Exception as e:
        print(f"Error in direct vision sensor test: {e}")
    
    print("\n=== GetDetectedObjects Test ===")
    # Test the GetDetectedObjects method
    try:
        items, packing_bay, obstacles, row_markers, shelves = robot.GetDetectedObjects()
        print("GetDetectedObjects completed successfully")
        print(f"Items detected: {items}")
        print(f"Packing bay: {packing_bay}")
        print(f"Obstacles: {obstacles}")
        print(f"Row markers: {row_markers}")
        print(f"Shelves: {shelves}")
        
    except Exception as e:
        print(f"Error in GetDetectedObjects: {e}")
        import traceback
        traceback.print_exc()
    
    print("\nStopping simulator...")
    robot.StopSimulator()
    
except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()
