# Import the required libraries
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
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

        self.cam = PiCamera()
        self.imWidth = 640
        self.imHeight = 480
        self.cam.resolution = (self.imWidth, self.imHeight)  # pixels
        self.rawCapture = PiRGBArray(self.cam, size=(self.imWidth, self.imHeight))

        # allow camera to warm up
        while self.cam.analog_gain <= 1:
            time.sleep(0.1)
                

    def SaveFrames(self):
        for frame in self.cam.capture_continuous(self.rawCapture, format='bgr', use_video_port=True):
            # format bgr for opencv
            im = frame.array

            # display image/wait for keypress
            cv2.imshow('picam frame', im)
            # k = cv.waitKey(1) & 0xFF

            # clear the stream in preparation for next frame
            self.rawCapture.truncate(0)

            # Increment loop counter
            self.loop = self.loop+1
            # Read keypress
            key = cv2.waitKey(1)
            # If the key was 'q' --> QUIT
            if key & 0xFF == ord('q'):            
                break
            # If the user has not pressed 'q' and the desired frequency has been reached --> SAVE FRAME
            elif self.function_type=="count" and self.loop % self.input_var==0:
                # Increment frame counter
                self.counter = self.counter+1 
                # Generate image path and filename
                string_name = "frames/frame_%06d.png"%self.counter
                # Write the frame to a png file (assumes there is a folder next to this script named "frames")
                cv2.imwrite(string_name, im)
            # If the user has not pressed 'q' and a key has been pressed --> SAVE FRAME
            elif self.function_type=="keypress" and key & 0xFF == ord(self.input_var):
                # Increment frame self.counter
                self.counter = self.counter+1 
                # Generate image path and filename
                string_name = "frames/frame_%06d.png"%self.counter
                # Write the frame to a png file (assumes there is a folder next to this script named "frames")
                cv2.imwrite(string_name, im)

    def CleanUp(self):
        # Release the camera object and close the window
        cv2.destroyAllWindows()



if __name__ == '__main__':
    # record on the keypress c
    image_recorder = ImageRecorder("keypress",'c')
    image_recorder.SaveFrames()
    image_recorder.CleanUp()