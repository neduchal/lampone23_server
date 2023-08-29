import cv2
import numpy as np


frame = cv2.imread("/var/www/html/image/image.png")


image = frame[260:761, 652:1328, :]

cv2.imwrite("/var/www/html/image/test.jpg", image)

