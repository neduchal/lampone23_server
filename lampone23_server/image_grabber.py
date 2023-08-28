import rclpy
from rclpy.node import Node
# dependencies rclpy image_transport cv_bridge sensor_msgs std_msgs opencv2
import cv2
import cv2.aruco
from cv_bridge import CvBridge 
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, String # For trigger message
import numpy as np
import time
#import scipy
#import skimage
#from skimage import transform as tf


class ImageGrabber(Node):

    def __init__(self):
        super().__init__('image_grabber')
        self.image_publisher = self.create_publisher(Image, "/image", 10)
        #self.timer = self.create_timer(5, self.image_saver_callback)
        self.cap =  cv2.VideoCapture(f'nvarguscamerasrc sensor-mode=3 ! video/x-raw(memory:NVMM), width=1920, height=1080, format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, width=(int)1920, height=(int)1080, format=(string)BGRx ! videoconvert ! appsink')
        self.image = None
        self.bridge = CvBridge()
        self.current_time = time.time()

        if not self.cap.isOpened():
            print("Cannot open camera")
            exit()

    def run(self):
        while True:
            print("OK")
            ret, frame = self.cap.read()

            # if frame is read correctly ret is True
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break
            if time.time() - self.current_time > 5:
                cv2.imwrite("/var/www/html/image/image.png", frame)
                self.current_time = time.time()

            self.image = frame
            self.image_publisher.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

    def image_saver_callback(self):
        if self.image is not None:
            cv2.imwrite("/var/www/html/image/image.png", self.image)

def main(args=None):
    rclpy.init(args=args)

    grabber = ImageGrabber()

    grabber.run()

    rclpy.spin(grabber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    grabber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()