#!/usr/bin/env python3

"""
Test script for verifying the fixes to proximity sensor and motor velocity issues
"""

from warehousebot_lib import *
import time

# Robot Parameters
robotParameters = RobotParameters()
robotParameters.driveType = 'differential'
robotParameters.sync = False

# Scene Parameters
sceneParameters = SceneParameters()

print("Testing fixes for proximity sensor and motor velocity errors...")
try:
    # Create robot object
    robot = COPPELIA_WarehouseRobot('127.0.0.1', robotParameters, sceneParameters)
    
    print("Starting simulator...")
    robot.StartSimulator()
    
    # Wait for simulation to settle
    time.sleep(2)
    
    print("\n=== Testing Proximity Sensor ===")
    for i in range(3):
        try:
            distance = robot.readProximity()
            print(f"  Proximity reading {i+1}: {distance:.3f} meters")
        except Exception as e:
            print(f"  Error reading proximity sensor: {e}")
        
        time.sleep(0.5)
    
    print("\n=== Testing Motor Velocities ===")
    test_velocities = [
        (0.1, 0.0),    # Forward
        (0.0, 0.5),    # Turn right
        (-0.1, 0.0),   # Backward
        (0.0, -0.5),   # Turn left
        (0.0, 0.0)     # Stop
    ]
    
    for i, (x_dot, theta_dot) in enumerate(test_velocities):
        try:
            print(f"  Setting velocity {i+1}: x_dot={x_dot}, theta_dot={theta_dot}")
            robot.SetTargetVelocities(x_dot, theta_dot)
            print(f"    Success!")
        except Exception as e:
            print(f"    Error setting velocities: {e}")
        
        time.sleep(1)
    
    print("\n=== Testing Object Detection ===")
    try:
        items, packing_bay, obstacles, row_markers, shelves = robot.GetDetectedObjects(
            [warehouseObjects.packingBay, warehouseObjects.obstacles]
        )
        print(f"  Detection successful - Packing bay: {packing_bay}, Obstacles: {obstacles}")
    except Exception as e:
        print(f"  Error in object detection: {e}")
    
    print("\nStopping simulator...")
    robot.StopSimulator()
    print("Test completed successfully!")
    
except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()
