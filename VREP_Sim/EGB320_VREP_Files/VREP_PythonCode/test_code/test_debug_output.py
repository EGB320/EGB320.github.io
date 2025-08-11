#!/usr/bin/python
"""
Test script to demonstrate the debug range and bearing output functionality
This script shows how the debug function handles different types of detection data
"""

import math

def print_debug_range_bearing(object_type, range_bearing_data):
	"""Debug function to print range and bearing information for detected objects
	
	Args:
		object_type (str): Name/description of the object type being displayed
		range_bearing_data: Range and bearing data (can be single [range, bearing] or list of such pairs)
	"""
	if range_bearing_data is None:
		print(f"🔍 {object_type}: No objects detected")
		return
	
	if isinstance(range_bearing_data, list):
		if len(range_bearing_data) == 0:
			print(f"🔍 {object_type}: Empty list (no detections)")
			return
		
		# Check if this is a single [range, bearing] pair or a list of pairs
		if len(range_bearing_data) == 2 and isinstance(range_bearing_data[0], (int, float)) and isinstance(range_bearing_data[1], (int, float)):
			# Single detection with range and bearing
			range_m = range_bearing_data[0]
			bearing_rad = range_bearing_data[1]
			bearing_deg = math.degrees(bearing_rad)
			print(f"🔍 {object_type}: Range = {range_m:.3f}m, Bearing = {bearing_rad:.3f}rad ({bearing_deg:.1f}°)")
		else:
			# List of multiple detections
			for i, rb in enumerate(range_bearing_data):
				if rb is not None and isinstance(rb, list) and len(rb) >= 2:
					range_m = rb[0]
					bearing_rad = rb[1]
					bearing_deg = math.degrees(bearing_rad)
					print(f"🔍 {object_type}[{i}]: Range = {range_m:.3f}m, Bearing = {bearing_rad:.3f}rad ({bearing_deg:.1f}°)")
				elif rb is None:
					print(f"🔍 {object_type}[{i}]: None (not detected)")
				else:
					print(f"🔍 {object_type}[{i}]: Invalid data - {rb}")
	else:
		print(f"🔍 {object_type}: Invalid format - {range_bearing_data}")

if __name__ == '__main__':
	print("🧪 Testing Debug Range and Bearing Output Functionality\n")
	
	# Test Case 1: No objects detected (None)
	print("Test Case 1: No objects detected")
	print_debug_range_bearing("Items", None)
	print()
	
	# Test Case 2: Empty list
	print("Test Case 2: Empty detection list")
	print_debug_range_bearing("Items", [])
	print()
	
	# Test Case 3: Single object detection
	print("Test Case 3: Single object detection")
	print_debug_range_bearing("Goal Waypoint", [1.234, 0.785])  # ~45 degrees
	print()
	
	# Test Case 4: Multiple objects with some None values
	print("Test Case 4: Multiple picking stations (some detected, some not)")
	picking_stations = [
		[0.567, -0.524],  # ~-30 degrees
		None,             # Not detected
		[2.345, 1.047],   # ~60 degrees
		None,             # Not detected
		[0.891, 0.000],   # 0 degrees (straight ahead)
		[1.500, -1.571],  # ~-90 degrees (left side)
	]
	print_debug_range_bearing("Picking Stations", picking_stations)
	print()
	
	# Test Case 5: Invalid data formats
	print("Test Case 5: Invalid data formats")
	print_debug_range_bearing("Invalid Object", [1])  # Too short
	print_debug_range_bearing("Another Invalid", "not a list")
	print()
	
	print("✅ Debug function testing complete!")
	print("\nExample usage in your robot code:")
	print("```python")
	print("# Get detected objects")
	print("itemsRB = warehouseBotSim.GetDetectedObjects()")
	print("print_debug_range_bearing('Items', itemsRB)")
	print("```")
