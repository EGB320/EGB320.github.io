#!/usr/bin/env python3
"""
Simple ZMQ Remote API Connection Test
This script tests the basic ZMQ connection to CoppeliaSim to help debug hanging issues.
"""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import signal
import threading
import sys

def test_zmq_connection():
    print("=" * 50)
    print("CoppeliaSim ZMQ Remote API Connection Test")
    print("=" * 50)
    
    def timeout_handler():
        print("\n⏰ TIMEOUT: Connection test is taking too long!")
        print("This usually means CoppeliaSim is not running the ZMQ Remote API server.")
        print("\nTo fix this:")
        print("1. Make sure CoppeliaSim is running")
        print("2. Load a scene file (.ttt) in CoppeliaSim")
        print("3. The ZMQ Remote API should start automatically in modern CoppeliaSim versions")
        print("4. If still not working, try restarting CoppeliaSim")
        print("\nForce closing Python script...")
        import os
        os._exit(1)
    
    # Set up a timeout to prevent infinite hanging
    timeout_thread = threading.Timer(10.0, timeout_handler)
    timeout_thread.start()
    
    try:
        # Step 1: Create client connection
        print("1. Creating RemoteAPIClient...")
        print("   - Attempting connection to localhost:23001 (CoppeliaSim detected port)")
        client = RemoteAPIClient(host='localhost', port=23001)
        print("   ✓ Client created successfully")
        
        # Step 2: Get sim object (this is where it often hangs)
        print("2. Getting sim object...")
        print("   - This step may hang if CoppeliaSim ZMQ server is not running...")
        sim = client.require('sim')
        print("   ✓ Sim object acquired")
        
        # Cancel the timeout since we got this far
        timeout_thread.cancel()
        
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
        timeout_thread.cancel()
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
    print("🔧 TROUBLESHOOTING CHECKLIST - Do this BEFORE running the test:")
    print("1. ✅ CoppeliaSim is running")
    print("2. ✅ A scene file (.ttt) is loaded in CoppeliaSim")
    print("3. ✅ Check CoppeliaSim console for ZMQ server startup messages")
    print("4. ✅ ZMQ Remote API should start automatically (no manual setup needed)")
    print("")
    
    input("Press Enter when you've verified the above checklist items...")
    
    success = test_zmq_connection()
    
    if success:
        print("\nNext steps:")
        print("- Try running your warehousebot code with sync=False")
        print("- Check that all object paths in your scene match what the code expects")
        print("- Use the verbose output to see where exactly it hangs")
    else:
        print("\nFix the connection issues above before proceeding.")
        print("\nAdditional debugging:")
        print("- Check CoppeliaSim's console/log for error messages")
        print("- Try a different scene file")
        print("- Restart CoppeliaSim completely")
        print("- Check if another process is using port 23000:")
