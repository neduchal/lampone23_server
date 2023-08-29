import cv2
import numpy as np

cap =  cv2.VideoCapture(f'nvarguscamerasrc sensor-mode=3 ! video/x-raw(memory:NVMM), width=1920, height=1080, format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, width=(int)1920, height=(int)1080, format=(string)BGRx ! videoconvert ! appsink')
#cap =  cv2.VideoCapture(f'nvarguscamerasrc sensor-mode=3 ! video/x-raw(memory:NVMM), width=3264, height=2464, format=(string)NV12, framerate=(fraction)21/1 ! nvvidconv ! video/x-raw, width=(int)3264, height=(int)2464, format=(string)BGRx ! videoconvert ! appsink')
if not cap.isOpened():
    print("Cannot open camera")
    exit()


while True:

    # Capture frame-by-frame
    ret, frame = cap.read()

    image = frame[260:761, 652:1328, :]

    cv2.imwrite("/var/www/html/image/test.png", image)

    # orez

    break