import rclpy
from rclpy.node import Node
# dependencies rclpy image_transport cv_bridge sensor_msgs std_msgs opencv2
import cv2
import cv2.aruco

from cv_bridge import CvBridge 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, String # For trigger message
import numpy as np
import time


class LamponeServerRobotController(Node):

    def __init__(self):
        super().__init__('lampone_server_robot_controller')
        self.solution_subscriber = self.create_subscription(
            String,
            '/lampone23/path',
            self.path_callback,
            10)
        self.solution_subscriber
        self.cap =  cv2.VideoCapture(f'nvarguscamerasrc sensor-mode=3 ! video/x-raw(memory:NVMM), width=1920, height=1080, format=(string)NV12, framerate=(fraction)29/1 ! nvvidconv ! video/x-raw, width=(int)1920, height=(int)1080, format=(string)BGRx ! videoconvert ! appsink')
        """
        self.image_subscriber = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            10)
        self.image_subscriber
        """

        if not self.cap.isOpened():
            print("Cannot open camera")
            exit()


        self.trigger_subscriber = self.create_subscription(
            Empty,
            '/lampone23/trigger',
            self.trigger_callback,
            10)
        self.trg = False
        self.trigger_subscriber
        self.twist_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.image = None
        timer_period = 30 # seconds
        self.timer = self.create_timer(timer_period, self.save_callback)
        self.path = []
        self.bridge = CvBridge()
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.arucoId = 1
        self.arucoParams = cv2.aruco.DetectorParameters_create()
        self.cells = []
        self.move = Twist()
        self.current_path = ""
        self.last_state = None
        timer_period2 = 0.1 # seconds
        self.timer = self.create_timer(timer_period2, self.control_callback)
        for i in range(8):
            for j in range(8):
                self.cells.append([i, j, int(55 + i/7 * (445-55)), int(75 + j/7 * (595 - 75))])
        self.size = 8
        self.current_time = time.time()

    def save_callback(self):
        ret, frame = self.cap.read()
        self.image = frame.copy()[260:761, 652:1328, :]
        cv2.imwrite("/var/www/html/image/image.png", frame)

    def control_callback(self):
        if len(self.path) > 0 and self.trg == True:
            ret, frame = self.cap.read()
            self.image = frame.copy()[260:761, 652:1328, :]
            if len(self.current_path) > 0:
                current_move = self.current_path[0]
                current_state = self.get_robot_position(self.last_state[0:2])
                if current_state is None:
                    move_msg = Twist()
                    self.twist_publisher.publish(move_msg)                 
                    return                      
                print(current_state)
                # Porovnat current a last state zda doslo ke správnému posunu.
                if self.is_move_complete(last_state=self.last_state, current_state=current_state, move=current_move):
                    print("ouha")
                    move_msg = Twist()
                    self.twist_publisher.publish(move_msg) 
                    self.last_state = current_state
                    if len(self.current_path) > 1:
                        self.current_path[1:]
                    else:
                        self.current_path = ""
                    return
                move_msg = Twist()
                if current_move == "L":
                    move_msg.angular.z = -0.7
                elif current_move == "R":
                    move_msg.angular.z = 0.7
                elif current_move == "F":
                    move_msg.linear.x = 0.75
                    last_angle = self.last_state[2]
                    current_angle = current_state[2]
                    diff = 0
                    if last_angle > 350 or last_angle < 10:
                        pass
                    elif last_angle > 80 and last_angle < 100:
                        diff = 90 - current_angle
                    elif last_angle > 170 and last_angle < 190:
                        diff = 180 - current_angle
                    elif last_angle > 260 and last_angle < 280:
                        diff = 270 - current_angle
                    if diff > 3:
                        move_msg.angular.z = 0.1
                    elif diff < -3:
                        move_msg.angular.z = -0.1
                elif current_move == "B":
                    move_msg.linear.x = -0.75

                    if last_angle > 350 or last_angle < 10:
                        pass
                    elif last_angle > 80 and last_angle < 100:
                        diff = 90 - current_angle
                    elif last_angle > 170 and last_angle < 190:
                        diff = 180 - current_angle
                    elif last_angle > 260 and last_angle < 280:
                        diff = 270 - current_angle
                    if diff > 3:
                        move_msg.angular.z = -0.1
                    elif diff < 3:
                        move_msg.angular.z = 0.1                
                else:
                    # DO NOTHING
                    pass

                self.twist_publisher.publish(move_msg)
                # Poslani zpravy na zastaveni
            else:
                self.trg = False
    


    def get_robot_position_px(self, markerCorner):
        # extract the marker corners (which are always returned in
        # top-left, top-right, bottom-right, and bottom-left order)
        corners = markerCorner.reshape((4, 2))
        (topLeft, topRight, bottomRight, bottomLeft) = corners
        # convert each of the (x, y)-coordinate pairs to integers
        topRight = (int(topRight[0]), int(topRight[1]))
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))

        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        cY = int((topLeft[1] + bottomRight[1]) / 2.0)
        u_vec = [topLeft[0] - bottomLeft[0], topLeft[1] - bottomLeft[1]]
        u_vec = np.array(u_vec / np.linalg.norm(u_vec))
        angle = np.arctan2(-u_vec[1], -u_vec[0]) # Je to -pi az pi. 0 kdyz je kod nahoru
        return np.array([cX, cY, angle, -u_vec[0], -u_vec[1]])

    def get_robot_position_in_grid(self, cells, robot_position, last_position):
        min_dist = 100000
        pos = [-1,-1]
        for i, cell in enumerate(cells):
            x = robot_position[0] - cell[3]
            y = robot_position[1] - cell[2]
            #print(x, y, robot_position[0:2])
            if np.linalg.norm([x,y]) < np.linalg.norm(min_dist):
                min_dist = np.array([x,y])
                pos = [cell[0], cell[1]]
        if pos[0] == -1 and pos[1] == -1:
            pos = last_position 
        return np.array([pos[0], pos[1], robot_position[2], robot_position[3], robot_position[4]])

    def path_callback(self, data):
        self.path.append(data.data)
        print(self.path)
        ret, frame = self.cap.read()
        self.image = frame.copy()[260:761, 652:1328, :]
        self.current_path = data.data
        if self.last_state is None:
            self.last_state = self.get_robot_position([-1, -1])

    """
    def image_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data)[260:761, 652:1328, :]
        print("IMAGE")
        pass
    """

    def get_robot_position(self, last_position):
        """
            Vraci pozici robota X,Y v ramci mrizky a jeho natoceni na zaklade cteni ARUCO tagu.
        """
        img = self.image.copy()

        (corners, ids, rejected) = cv2.aruco.detectMarkers(img, self.arucoDict,
            parameters=self.arucoParams)
        # verify *at least* one ArUco marker was detected
        robot_pos_grid = None
        print(corners)
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()            
            for (markerCorner, markerID) in zip(corners, ids):
                if markerID == self.arucoId:
                    robot_position = self.get_robot_position_px(markerCorner)
                    robot_pos_grid = self.get_robot_position_in_grid(self.cells, robot_position, last_position)
        return robot_pos_grid

    def is_move_complete(self, last_state, current_state, move):
        state = current_state - last_state

        last_vec = last_state[3:]
        current_vec = current_state[3:]
        #if last_vec[0] == current_vec[0] and last_vec[1] == current_vec[1]:
        #    angle = last_state[2]
        angle = 180 * np.arccos(current_vec.dot(last_vec))/np.pi
        if (current_state[2] - last_state[2]) < 0 or np.abs(last_state[2]- current_state[2]) > 180:
            angle = -angle

        if move == "L":
            if angle  > 87 and angle < 93:
                return True
        elif move == "R":
            if angle  > -93 and angle < -87:
                return True       
        elif move == "F":
            if (last_state[2] > 355) or (last_state[2] < 5):
                if state[0] == 1:
                    return True
            elif (last_state[2] > 85) and (last_state[2] < 95):
                if state[1] == 1:
                    return True
            elif (last_state[2] > 175) or (last_state[2] < 185):
                if state[0] == -1:
                    return True
            elif (last_state[2] > 265) or (last_state[2] < 275):
                if state[1] == -1:
                    return True
        elif move == "B":
            if (last_state[2] > 355) or (last_state[2] < 5):
                if state[0] == -1:
                    return True
            elif (last_state[2] > 85) and (last_state[2] < 95):
                if state[1] == -1:
                    return True
            elif (last_state[2] > 175) or (last_state[2] < 185):
                if state[0] == 1:
                    return True
            elif (last_state[2] > 265) or (last_state[2] < 275):
                if state[1] == 1:
                    return True
            pass
        else:
            return True
        return False

    """
    def process_path(self, path):
        for current_move in path:
            last_state = self.get_robot_position([-1, -1])
            if last_state[0] < 0 or last_state[1] < 0 or last_state[0] > self.size -1 or last_state[1] > self.size -1:
                move_msg = Twist()
                self.twist_publisher.publish(move_msg)
                break
            last_time = time.time()
    """
   

    """
    def run_callback(self):
        if time.time() > self.current_time + 1:
            if len(self.path) > 0 and self.trg == True:
                self.process_path(self.path.pop(0))
                self.trg = False
            self.current_time = time.time()

    """
    
    def trigger_callback(self, msg):
        self.trg = True
        ret, frame = self.cap.read()
        self.image = frame.copy()[260:761, 652:1328, :]
        print("Trigger")

def main(args=None):
    rclpy.init(args=args)

    controller = LamponeServerRobotController()

    #controller.run()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()