#!/usr/bin/python

"""
Performance optimization test for ZMQ API calls
This script tests different strategies to improve loop performance
"""

from warehousebot_lib import *
import time
from collections import defaultdict

# Timing tracking variables
timing_stats = defaultdict(list)

def benchmark_api_calls():
    print("🚀 ZMQ API Performance Optimization Test")
    print("Testing different strategies to improve loop performance\n")
    
    # Robot Parameters
    robotParameters = RobotParameters()
    robotParameters.driveType = 'differential'
    robotParameters.sync = False

    # Scene Parameters  
    sceneParameters = SceneParameters()
    
    try:
        # Create robot
        robot = COPPELIA_WarehouseRobot('127.0.0.1', robotParameters, sceneParameters)
        robot.StartSimulator()
        time.sleep(2)  # Let simulation settle
        
        print("📊 Test 1: Minimal API calls (just motor control)")
        start_time = time.perf_counter()
        for i in range(10):
            robot.SetTargetVelocities(0.0, 0.1)
            time.sleep(0.01)  # Small delay
        minimal_time = (time.perf_counter() - start_time) / 10
        print(f"   Average time per loop: {minimal_time*1000:.1f}ms ({1000/minimal_time:.1f} FPS)")
        
        print("\n📊 Test 2: Motor + Proximity only")
        start_time = time.perf_counter()
        for i in range(10):
            robot.SetTargetVelocities(0.0, 0.1)
            robot.readProximity()
            time.sleep(0.01)
        motor_prox_time = (time.perf_counter() - start_time) / 10
        print(f"   Average time per loop: {motor_prox_time*1000:.1f}ms ({1000/motor_prox_time:.1f} FPS)")
        
        print("\n📊 Test 3: Full detection (current implementation)")
        start_time = time.perf_counter()
        for i in range(5):  # Fewer iterations since it's slow
            robot.SetTargetVelocities(0.0, 0.1)
            robot.GetDetectedObjects()
            robot.readProximity()
            robot.UpdateObjectPositions()
            time.sleep(0.01)
        full_time = (time.perf_counter() - start_time) / 5
        print(f"   Average time per loop: {full_time*1000:.1f}ms ({1000/full_time:.1f} FPS)")
        
        print("\n📊 Test 4: Selective detection (items + obstacles only)")
        start_time = time.perf_counter()
        for i in range(10):
            robot.SetTargetVelocities(0.0, 0.1)
            robot.GetDetectedObjects([warehouseObjects.items, warehouseObjects.obstacles])
            robot.readProximity()
            time.sleep(0.01)
        selective_time = (time.perf_counter() - start_time) / 10
        print(f"   Average time per loop: {selective_time*1000:.1f}ms ({1000/selective_time:.1f} FPS)")
        
        print("\n🎯 Performance Optimization Recommendations:")
        print(f"1. Minimal loop (motor only): {1000/minimal_time:.1f} FPS")
        print(f"2. Basic sensors (motor+proximity): {1000/motor_prox_time:.1f} FPS") 
        print(f"3. Selective detection: {1000/selective_time:.1f} FPS")
        print(f"4. Full detection: {1000/full_time:.1f} FPS")
        
        print(f"\n💡 Suggestions:")
        print(f"   - Consider running full detection every N loops instead of every loop")
        print(f"   - Use selective object detection based on current task")
        print(f"   - Skip UpdateObjectPositions if not needed every loop")
        print(f"   - Implement async detection or multi-threading for sensor calls")
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
    finally:
        try:
            robot.StopSimulator()
            print("\n✅ Test completed")
        except:
            pass

if __name__ == "__main__":
    benchmark_api_calls()
