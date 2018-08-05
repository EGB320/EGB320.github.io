import numpy as np
import cv2

frame = cv2.imread("../../../images/Vision/frame_000191.png")							# Import the raw frame
frame_blue = frame[:,:,0];								# Extract the blue colour channel
b = frame[:,:,0]
g = frame[:,:,1]
r = frame[:,:,2]
cv2.imshow("Red", r)	
cv2.imshow("Blue", b)	
cv2.imshow("Green", g)	
ret, thresholded_frame = cv2.threshold(frame_blue, 127, 255, cv2.THRESH_BINARY)		# Threshold blue channel

cv2.imshow("Binary Thresholded Frame", thresholded_frame)				# Display the thresholded frame
cv2.waitKey(0)										# Wait for a keypress before exiting

cap.release()
cv2.destroyAllWindows()