
import numpy as np
import cv2
import cv2.aruco

cap =  cv2.VideoCapture(f'nvarguscamerasrc sensor-mode=3 ! video/x-raw(memory:NVMM), width=3264, height=2464, format=(string)NV12, framerate=(fraction)21/1 ! nvvidconv ! video/x-raw, width=(int)3264, height=(int)2464, format=(string)BGRx ! videoconvert ! appsink')
if not cap.isOpened():
    print("Cannot open camera")
    exit()

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
arucoId = 1
arucoParams = cv2.aruco.DetectorParameters_create()    
i = 0
while True:
    if i > 3000:
        break
    i += 1
    # Capture frame-by-frame
    ret, frame = cap.read()
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Display the resulting frame
    (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
    # verify *at least* one ArUco marker was detected
    if len(corners) > 0:
        # flatten the ArUco IDs list
        ids = ids.flatten()            
        for (markerCorner, markerID) in zip(corners, ids):
            if markerID == arucoId:    
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                u_vec = [bottomLeft[0] - bottomRight[0], bottomLeft[1] - bottomRight[1]]
                u_vec = np.array(u_vec / np.linalg.norm(u_vec))
                angle = np.arctan2(u_vec[0], u_vec[1]) # Je to -pi az pi . zkontrolovat zda s tim pocita i zbytek kodu          
                print (cX, cY, angle)
    else:
        print("None")
    #cv2.imwrite("test.png", gray)

# When everything done, release the capture
cap.release()







