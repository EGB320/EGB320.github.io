# Import the required libraries
import numpy as np
import cv2

# Create the video capture object
cap = cv2.VideoCapture(0)

# Initialise variables
frames = []
counter = 0


def SaveEveryNthFrame(freq):
    # Define a separate counter to determine the loop number
    loop = 0
    # Complete the following loop while the video capture object is active
    while(cap.isOpened()):
        # Read the frame
        ret, frame = cap.read()
        # If a frame has been obtained, complete the following
        if ret==True:        
            # Display the frame
            cv2.imshow('frame',frame)
            # Increment loop counter
            loop = loop+1
            # Read keypress
            key = cv2.waitKey(1)
            # If the key was 'q' --> QUIT
            if key & 0xFF == ord('q'):            
                break
            # If the user has not pressed 'q' --> SAVE FRAME
            else if loop%freq:
                # Increment frame counter
                counter = counter+1 
                # Generate image path and filename
                string_name = "frames/frame_%06d.png"%counter
                # Write the frame to a png file (assumes there is a folder next to this script named "frames")
                cv2.imwrite(string_name, frame)
        else:
            break

def SaveFrameOnKeyPress():
    # Complete the following loop while the video capture object is active
    while(cap.isOpened()):
        # Read the frame
        ret, frame = cap.read()
        # If a frame has been obtained, complete the following
        if ret==True:        
            # Display the frame
            cv2.imshow('frame',frame)
            # Read keypress
            key = cv2.waitKey(1)
            # If the key was 'q' --> QUIT
            if key & 0xFF == ord('q'):            
                break
            # If the key was 'a' --> SAVE FRAME
            elif key & 0xFF == ord('a'):
                # Increment frame counter
                counter = counter+1 
                # Generate image path and filename
                string_name = "frames/frame_%06d.png"%counter
                # Write the frame to a png file (assumes there is a folder next to this script named "frames")
                cv2.imwrite(string_name, frame)
        else:
            break



# Release the camera object and close the window
cap.release()
cv2.destroyAllWindows()