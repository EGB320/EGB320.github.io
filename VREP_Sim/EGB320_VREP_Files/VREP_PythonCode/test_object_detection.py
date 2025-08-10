#!/usr/bin/env python3
"""
Test script for the updated ObjectDetector implementation
"""

from warehousebot_lib import *
import time

def test_object_detection():
    print("Testing Object Detection with ZMQ Remote API...")
    
    # SET SCENE PARAMETERS
    sceneParameters = SceneParameters()
    sceneParameters.bayContents = np.random.random_integers(0,5,sceneParameters.bayContents.shape)
    sceneParameters.obstacle0_StartingPosition = [-0.5,0]
    sceneParameters.obstacle1_StartingPosition = -1
    sceneParameters.obstacle2_StartingPosition = -1

    # SET ROBOT PARAMETERS
    robotParameters = RobotParameters()
    robotParameters.sync = False  # Important: keep this False to avoid hanging
    
    try:
        # Create warehouse bot object
        print("Creating WarehouseBot object...")
        warehouseBotSim = COPPELIA_WarehouseRobot('127.0.0.1', robotParameters, sceneParameters)
        
        # Start simulator (or ensure it's running)
        print("Starting simulator...")
        warehouseBotSim.StartSimulator()
        
        print("Testing object detection for 5 cycles...")
        
        for i in range(5):
            print(f"\n--- Detection Cycle {i+1} ---")
            
            # Update object positions
            warehouseBotSim.UpdateObjectPositions()
            
            # Get detected objects
            objectsRB = warehouseBotSim.GetDetectedObjects([
                warehouseObjects.items,
                warehouseObjects.shelves,
                warehouseObjects.row_markers,
                warehouseObjects.obstacles,
                warehouseObjects.pickingStation,
            ])
            
            itemsRB, packingStationRB, obstaclesRB, rowMarkerRB, shelfRB = objectsRB
            
            # Print detection results
            print("Detection Results:")
            print(f"  Items detected: {[i for i, item in enumerate(itemsRB) if item is not None]}")
            print(f"  Shelves detected: {[i for i, shelf in enumerate(shelfRB) if shelf is not None]}")
            print(f"  Obstacles detected: {'Yes' if obstaclesRB else 'No'}")
            print(f"  Picking station detected: {'Yes' if packingStationRB else 'No'}")
            print(f"  Row markers detected: {[i for i, marker in enumerate(rowMarkerRB) if marker is not None]}")
            
            # Move robot slightly for testing
            warehouseBotSim.SetTargetVelocities(0.01, 0.05)
            
            time.sleep(2)
        
        print("\n✓ Object detection test completed successfully!")
        warehouseBotSim.StopSimulator()
        
    except Exception as e:
        print(f"❌ Test failed with error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_object_detection()
