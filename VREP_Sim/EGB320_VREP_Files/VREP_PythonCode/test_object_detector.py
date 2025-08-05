#!/usr/bin/env python3
"""
Test script to check ObjectDetector sensor output format
"""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

def test_object_detector():
    print("Testing ObjectDetector sensor...")
    
    try:
        # Connect to CoppeliaSim
        client = RemoteAPIClient(host='localhost', port=23000)
        sim = client.require('sim')
        
        # Get ObjectDetector handle
        try:
            object_detector_handle = sim.getObject('/Robot/ObjectDetector')
            print(f"ObjectDetector handle: {object_detector_handle}")
        except Exception as e:
            print(f"Could not get ObjectDetector handle: {e}")
            print("Available objects in /Robot:")
            try:
                robot_handle = sim.getObject('/Robot')
                children = sim.getObjectsInTree(robot_handle, sim.handle_all, 0)
                for child in children:
                    try:
                        name = sim.getObjectAlias(child)
                        print(f"  - {name} (handle: {child})")
                    except:
                        print(f"  - handle: {child} (name unavailable)")
            except Exception as e2:
                print(f"Could not list Robot children: {e2}")
            return False
        
        # Test the vision sensor
        print("\nTesting handleVisionSensor...")
        for i in range(3):
            try:
                result, data, _ = sim.handleVisionSensor(object_detector_handle)
                print(f"Attempt {i+1}:")
                print(f"  Result: {result}")
                print(f"  Data type: {type(data)}")
                print(f"  Data: {data}")
                
                if isinstance(data, list):
                    print(f"  Data length: {len(data)}")
                    if len(data) > 0:
                        print(f"  First element: {data[0]} (type: {type(data[0])})")
                
                time.sleep(0.5)
                
            except Exception as e:
                print(f"  Error: {e}")
        
        # Try reading vision sensor image/detection data differently
        print("\nTrying alternative vision sensor functions...")
        try:
            # Try getting vision sensor image
            img, resolution = sim.getVisionSensorImg(object_detector_handle)
            print(f"Vision sensor image resolution: {resolution}")
            print(f"Image data type: {type(img)}, length: {len(img) if hasattr(img, '__len__') else 'N/A'}")
        except Exception as e:
            print(f"getVisionSensorImg failed: {e}")
        
        try:
            # Try reading as detection sensor
            detected, point, handle, normal = sim.readVisionSensor(object_detector_handle)
            print(f"readVisionSensor - detected: {detected}, point: {point}, handle: {handle}")
        except Exception as e:
            print(f"readVisionSensor failed: {e}")
            
        return True
        
    except Exception as e:
        print(f"Test failed: {e}")
        return False

if __name__ == "__main__":
    success = test_object_detector()
    if not success:
        print("\nMake sure:")
        print("1. CoppeliaSim is running")
        print("2. The warehouse scene is loaded")
        print("3. The /Robot/ObjectDetector object exists in the scene")
