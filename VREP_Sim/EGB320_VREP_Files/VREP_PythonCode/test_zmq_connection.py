#!/usr/bin/env python3
"""
Simple ZMQ Remote API Connection Test
This script tests the basic ZMQ connection to CoppeliaSim to help debug hanging issues.
"""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

def test_zmq_connection():
    print("=" * 50)
    print("CoppeliaSim ZMQ Remote API Connection Test")
    print("=" * 50)
    
    try:
        # Step 1: Create client connection
        print("1. Creating RemoteAPIClient...")
        client = RemoteAPIClient(host='localhost', port=23000)
        print("   ✓ Client created successfully")
        
        # Step 2: Get sim object
        print("2. Getting sim object...")
        sim = client.require('sim')
        print("   ✓ Sim object acquired")
        
        # Step 3: Test basic sim calls
        print("3. Testing basic sim calls...")
        
        # Get simulation time
        sim_time = sim.getSimulationTime()
        print(f"   ✓ Simulation time: {sim_time}")
        
        # Get simulation state
        sim_state = sim.getSimulationState()
        print(f"   ✓ Simulation state: {sim_state}")
        
        # Try to get a simple object (if it exists)
        print("4. Testing object retrieval...")
        try:
            # Test getting a common object - this might fail if object doesn't exist
            robot_handle = sim.getObject('/Robot')
            print(f"   ✓ Found Robot object with handle: {robot_handle}")
        except Exception as e:
            print(f"   - Robot object not found (this is ok): {e}")
        
        # List all objects in scene
        print("5. Listing scene objects...")
        try:
            all_objects = sim.getObjectsInTree(sim.handle_scene)
            print(f"   ✓ Found {len(all_objects)} objects in scene")
            
            # Get names of first few objects
            for i, obj_handle in enumerate(all_objects[:5]):
                try:
                    obj_name = sim.getObjectAlias(obj_handle)
                    print(f"   - Object {i}: {obj_name} (handle: {obj_handle})")
                except:
                    print(f"   - Object {i}: handle {obj_handle} (name unavailable)")
        except Exception as e:
            print(f"   - Could not list objects: {e}")
        
        print("\n" + "=" * 50)
        print("✓ CONNECTION TEST PASSED")
        print("The ZMQ Remote API connection is working properly.")
        print("If your code is still hanging, the issue is likely:")
        print("1. Trying to access objects that don't exist in the scene")
        print("2. Using synchronous stepping mode (setStepping(True))")
        print("3. Scene-specific object naming or structure issues")
        print("=" * 50)
        
        return True
        
    except Exception as e:
        print(f"\n❌ CONNECTION TEST FAILED")
        print(f"Error: {e}")
        print("\nPossible solutions:")
        print("1. Make sure CoppeliaSim is running")
        print("2. Make sure a scene is loaded in CoppeliaSim")
        print("3. Check that ZMQ Remote API is enabled (default in modern versions)")
        print("4. Verify port 23000 is not blocked by firewall")
        print("5. Try restarting CoppeliaSim")
        return False

if __name__ == "__main__":
    success = test_zmq_connection()
    
    if success:
        print("\nNext steps:")
        print("- Try running your warehousebot code with sync=False")
        print("- Check that all object paths in your scene match what the code expects")
        print("- Use the verbose output to see where exactly it hangs")
    else:
        print("\nFix the connection issues above before proceeding.")
