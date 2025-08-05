#!/usr/bin/env python3

"""
Test script for verifying the updated GetShelfRangeBearing function using sim.checkDistance
"""

from warehousebot_lib import *
import time

# Robot Parameters
robotParameters = RobotParameters()
robotParameters.driveType = 'differential'
robotParameters.sync = False

# Scene Parameters
sceneParameters = SceneParameters()

print("Testing GetShelfRangeBearing with sim.checkDistance...")
try:
    # Create robot object
    robot = COPPELIA_WarehouseRobot('127.0.0.1', robotParameters, sceneParameters)
    
    print("Starting simulator...")
    robot.StartSimulator()
    
    # Wait for simulation to settle
    time.sleep(2)
    
    print("\n=== Testing Shelf Range and Bearing Detection ===")
    for i in range(3):
        try:
            print(f"\nTest cycle {i+1}:")
            
            # Test the shelf range and bearing function
            shelf_rb = robot.GetShelfRangeBearing()
            
            print(f"  Shelf range and bearing results:")
            for shelf_idx, rb in enumerate(shelf_rb):
                if rb is not None:
                    range_val, bearing = rb
                    print(f"    Shelf {shelf_idx}: Range={range_val:.3f}m, Bearing={bearing:.3f}rad")
                else:
                    print(f"    Shelf {shelf_idx}: Not detected")
                    
        except Exception as e:
            print(f"  Error in test cycle {i+1}: {e}")
        
        time.sleep(1)
    
    print("\n=== Testing Full Object Detection with Shelves ===")
    try:
        items, packing_bay, obstacles, row_markers, shelves = robot.GetDetectedObjects(
            [warehouseObjects.shelves, warehouseObjects.packingBay, warehouseObjects.obstacles]
        )
        
        print(f"  Shelves detected: {shelves}")
        print(f"  Packing bay: {packing_bay}")
        print(f"  Obstacles: {obstacles}")
        
        # Count detected shelves
        detected_shelves = sum(1 for shelf in shelves if shelf is not None)
        print(f"  Total shelves detected: {detected_shelves}")
        
    except Exception as e:
        print(f"  Error in full object detection: {e}")
    
    print("\nStopping simulator...")
    robot.StopSimulator()
    print("Test completed successfully!")
    
except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()
