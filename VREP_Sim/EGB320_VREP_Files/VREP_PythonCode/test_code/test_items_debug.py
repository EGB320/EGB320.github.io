#!/usr/bin/python
"""
Test the updated debug function with items array structure
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
	
	# Special handling for items array (6-element array with one element per item type)
	if object_type == "Items" and isinstance(range_bearing_data, list) and len(range_bearing_data) == 6:
		item_names = ["Bowls", "Mugs", "Bottles", "Soccer Balls", "Rubiks Cubes", "Cereal Boxes"]
		any_items_found = False
		
		for item_type, detections in enumerate(range_bearing_data):
			if detections is not None and len(detections) > 0:
				any_items_found = True
				for i, rb in enumerate(detections):
					if rb is not None and len(rb) >= 2:
						range_m = rb[0]
						bearing_rad = rb[1]
						bearing_deg = math.degrees(bearing_rad)
						print(f"🔍 {item_names[item_type]}[{i}]: Range = {range_m:.3f}m, Bearing = {bearing_rad:.3f}rad ({bearing_deg:.1f}°)")
		
		if not any_items_found:
			print(f"🔍 {object_type}: No items detected")
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
	print("🧪 Testing Items Array Debug Function\n")
	
	# Test Case 1: Items array with bowl detection (what you're experiencing)
	print("Test Case 1: Items array with bowl detection (your scenario)")
	items_array = [
		[[0.48277960888500576, 0.012764459010854878]],  # Bowl detected
		None,  # Mug not detected
		None,  # Bottle not detected
		None,  # Soccer ball not detected
		None,  # Rubiks cube not detected
		None   # Cereal box not detected
	]
	print_debug_range_bearing("Items", items_array)
	print()
	
	# Test Case 2: Multiple items detected
	print("Test Case 2: Multiple different items detected")
	items_array2 = [
		[[0.5, 0.1], [0.8, -0.2]],  # Two bowls detected
		[[1.2, 0.3]],               # One mug detected
		None,                       # Bottle not detected
		[[2.1, -0.5]],             # One soccer ball detected
		None,                       # Rubiks cube not detected
		[[0.9, 0.0]]               # One cereal box detected
	]
	print_debug_range_bearing("Items", items_array2)
	print()
	
	# Test Case 3: Other object types (unchanged behavior)
	print("Test Case 3: Other object types (normal behavior)")
	print_debug_range_bearing("Goal Waypoint", [1.234, 0.785])
	print_debug_range_bearing("Picking Stations", [[0.5, 0.1], None, [1.2, -0.3]])
	print()
	
	print("✅ All tests completed!")
