import numpy as np
import cv2

cap = cv2.VideoCapture(0)

# Define the codec and create VideoWriter object
fourcc = cv2.cv.CV_FOURCC(*'DIVX')
#out = cv2.VideoWriter('output.avi',fourcc, 20.0, (640,480))

#out = cv2.VideoWriter('output.avi', , 20.0, (640,480))
frames = []
counter = 0
while(cap.isOpened()):
    ret, frame = cap.read()
    if ret==True:        
        cv2.imshow('frame',frame)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):            
            break
        elif key & 0xFF == ord('a'):
            counter = counter+1
            string_name = "frames/frame_" + str(counter) +".png"
            cv2.imwrite(string_name, frame)
    else:
        break

# Release everything if job is finished
cap.release()
cv2.destroyAllWindows()