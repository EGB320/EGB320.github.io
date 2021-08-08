# Import the required libraries
import numpy as np
import cv2

class ImageRecorder():
    """Class to enable recording of images using python and opencv"""
    def __init__(self, function_type, input_var):
        # Initialise variables
        self.counter = 0
        self.input_okay = False
        self.loop = 0
        self.function_type = function_type
        self.input_var = input_var

        # Check parameters match
        if self.function_type=="keypress" and type(self.input_var)==str and len(self.input_var)==1:
                self.input_okay=True
        elif self.function_type=="count" and type(self.input_var)==int:
                self.input_okay=True
        else:
            print("Input sequence does not match requirements.")
            self.input_okay=False
            return

        # Create the video capture object
        self.cap = cv2.VideoCapture(0)

        # Set the resolution you want from your camera - change this for higher quality images (e.g. 640 x 480). 
        self.cap.set(3,320) # Width --> Channel 3
        self.cap.set(4,240) # Height --> Channel 4
        # Other camera parameters can be altered, such as brightness and contrast.
        # See https://docs.opencv.org/3.0-beta/modules/videoio/doc/reading_and_writing_video.html#videocapture-set
        # We are leaving all other parameters at default in this script.
        

    def SaveFrames(self):
        # Complete the following loop while the video capture object is active
        while(self.cap.isOpened()) and self.input_okay:
            # Read the frame`
            ret, frame = self.cap.read()
            # If a frame has been obtained, complete the following
            if ret==True:        
                # Display the frame
                cv2.imshow('frame',frame)
                # Increment loop counter
                self.loop = self.loop+1
                # Read keypress
                key = cv2.waitKey(1)
                # If the key was 'q' --> QUIT
                if key & 0xFF == ord('q'):            
                    break
                # If the user has not pressed 'q' and the desired frequency has been reached --> SAVE FRAME
                elif self.function_type=="count" and self.loop%self.input_var==0:
                    # Increment frame counter
                    self.counter = self.counter+1 
                    # Generate image path and filename
                    string_name = "frames/frame_%06d.png"%self.counter
                    # Write the frame to a png file (assumes there is a folder next to this script named "frames")
                    cv2.imwrite(string_name, frame)
                # If the user has not pressed 'q' and a key has been pressed --> SAVE FRAME
                elif self.function_type=="keypress" and key & 0xFF == ord(self.input_var):
                    # Increment frame self.counter
                    self.counter = self.counter+1 
                    # Generate image path and filename
                    string_name = "frames/frame_%06d.png"%self.counter
                    # Write the frame to a png file (assumes there is a folder next to this script named "frames")
                    cv2.imwrite(string_name, frame)
            else:
                break

    def CleanUp(self):
        # Release the camera object and close the window
        self.cap.release()
        cv2.destroyAllWindows()



if __name__ == '__main__':
    image_recorder = ImageRecorder("count",1000)
    image_recorder.SaveFrames()
    image_recorder.CleanUp()


