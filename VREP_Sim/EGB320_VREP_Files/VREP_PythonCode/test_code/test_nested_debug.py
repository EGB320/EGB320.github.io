#!/usr/bin/python
"""
Test the corrected debug function with the nested list structure
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
					# Check if this is a nested list like [[range, bearing]]
					if isinstance(rb[0], list) and len(rb[0]) >= 2:
						# Handle nested structure [[range, bearing]]
						inner_rb = rb[0]
						range_m = inner_rb[0]
						bearing_rad = inner_rb[1]
						bearing_deg = math.degrees(bearing_rad)
						print(f"🔍 {object_type}[{i}]: Range = {range_m:.3f}m, Bearing = {bearing_rad:.3f}rad ({bearing_deg:.1f}°)")
					else:
						# Normal [range, bearing] structure
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
	print("🧪 Testing Corrected Debug Function with Nested Lists\n")
	
	# Test the problematic case from the user
	print("Test Case: Nested list structure [[range, bearing]]")
	print_debug_range_bearing("Items", [[0.48277960888500576, 0.012764459010854878]])
	print()
	
	# Test normal case
	print("Test Case: Normal list structure [range, bearing]")
	print_debug_range_bearing("Goal", [1.234, 0.785])
	print()
	
	# Test multiple items
	print("Test Case: Multiple normal items")
	print_debug_range_bearing("Stations", [[0.5, 0.1], [1.2, -0.3], None, [2.1, 0.8]])
	print()
	
	print("✅ All tests completed!")
