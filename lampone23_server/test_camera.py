import numpy as np
import cv2


cap =  cv2.VideoCapture(f'nvarguscamerasrc sensor-mode=3 ! video/x-raw(memory:NVMM), width=1920, height=1080, format=(string)NV12, framerate=(fraction)29/1 ! nvvidconv ! video/x-raw, width=(int)1920, height=(int)1080, format=(string)BGRx ! videoconvert ! appsink')

#cap = cv2.VideoCapture(f'v4l2src device=/dev/video0 io-mode=2 ! image/jpeg, width=(int)1920, height=(int)1080 !  nvjpegdec ! video/x-raw, format=RG10 ! appsink', cv2.CAP_GSTREAMER)
if not cap.isOpened():
    print("Cannot open camera")
    exit()
i = 0
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    print(i)
    i += 1
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    # Our operations on the frame come here
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Display the resulting frame
    #cv2.imwrite("/var/www/html/image/image.png", frame)
    #break
# When everything done, release the capture
cap.release()
