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
import scipy
import skimage
from skimage import transform as tf

class LamponeServerRobotController(Node):

    def __init__(self):
        self.solution_subscriber = self.create_subscription(
            String,
            '/lampone23/path',
            self.path_callback,
            10)
        self.solution_subscriber
        self.image_subscriber = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            10)
        self.image_subscriber
        self.trigger_subscriber = self.create_subscription(
            Empty,
            'lampone23/trigger',
            self.trigger_callback,
            10)
        self.trigger_subscriber
        self.twist_publisher = self.create_publisher(TwistStamped, "cmd_vel", 10)
        self.image = None
        self.path = []
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.run_callback)
        self.bridge = CvBridge()
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
        self.arucoId = 1
        self.arucoParams = cv2.aruco.DetectorParameters_create()
        self.cells = None


    def transform_image(self, img, src, dst):
        tform3 = tf.ProjectiveTransform()
        tform3.estimate(dst, src)
        warped = tf.warp(img, tform3, output_shape=tuple(np.max(dst, axis=0)))
        return warped

    def normalize_img(self, img, mask_size=5):
        # Create a mask with blured image with the same value range
        mask =scipy.signal.convolve2d(img, np.ones((mask_size,mask_size))/(mask_size**2),mode='same',boundary='symm')

        # Substract and normalize to <0,1> range
        normalized = img-mask+0.5
        return normalized

    def nms(self, input, neighborhood_size=40):
        # Add padding with size equal to neighborhood size (so we dont lose information from the image edges)
        padding = neighborhood_size
        img_padded = np.pad(input, padding)

        # Prepare the result array
        result = np.zeros(input.shape)

        # Iterate through the image
        for i in range(input.shape[0]):
            i_ = i+padding
            for j in range(input.shape[1]):
                j_ = j+padding

            # Find maximum in the neighborhood
            max_val = np.max(img_padded[i_-neighborhood_size:i_+neighborhood_size,j_-neighborhood_size:j_+neighborhood_size])

            # Make the output array
            if max_val == img_padded[i_,j_]:
                result[i,j] = 1
        return result.T

    def get_grid(self, img):
        # Normalize image
        normalized = self.normalize_img(img)

        # Creating kernel with pattern
        kernel = np.ones((9,9))
        kernel[3:-3,:] = 0
        kernel[:,3:-3] = 0
        print('Kernel:')
        print(kernel)

        # Convolution with kernel
        convolution_output =scipy.signal.convolve2d(normalized, kernel, mode='same')

        # Finding the local maximums
        points = self.nms(convolution_output)
        
        print('Number of detected points:')
        print(np.sum(points))

        return np.where(points)

    # Find Corners of grid
    def get_new_src_coordinates(self, src, points, n_in_row=9, offset=20):
        points_min = np.min(points, axis=1)-offset
        points_max = np.max(points, axis=1)+offset
        src_new = np.array([points_min, [points_max[0], points_min[1]], [points_min[0], points_max[1]], points_max])
        return src_new

    def get_cell_centers(self, points, n_in_row=9):
        cells=[[],[]]
        points_coords = sorted(np.array(points).T.tolist(), key=lambda x: x[0])
        for i in range(n_in_row-1):
            for j in range(n_in_row-1):
                # Get first row
                row1 = points_coords[i*9:(i+1)*9]
                # Get first two elements from first row
                col1 = sorted(row1, key=lambda x: x[1])[j:2+j]
                # Get second row
                row2 = points_coords[9*(i+1):(i+2)*9]
                # Get first two elements from second row
                col2 = sorted(row2, key=lambda x: x[1])[j:2+j]
                corners = np.array(col1+col2).T
                y,x = np.mean(corners,axis=1)
                cells[0].append(x)
                cells[1].append(y)
        return cells

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
        u_vec = np.array(u_vec / np.norm(u_vec))
        angle = np.arctan2(-u_vec[1], -u_vec[2]) # Je to -pi az pi. 0 kdyz je kod nahoru
        return np.array([cX, cY, angle])

    def get_robot_position_in_grid(self, cells, robot_position, grid_size):
        min_dist = robot_position[0:2].copy()
        pos = [-1,-1]
        for i, cell in enumerate(cells):
            x = robot_position[0] - cell[0]
            y = robot_position[1] - cell[1]
            if np.norm([x,y]) < np.norm(min_dist):
                min_dist = np.array([x,y])
                pos = [i % grid_size, i // grid_size]
        return np.array([pos[0], pos[1], robot_position[2]])

    def path_callback(self, data):
        self.path.append(data.data)
        pass
    
    def image_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data)
        pass

    def get_robot_position(self):
        """
            Vraci pozici robota X,Y v ramci mrizky a jeho natoceni na zaklade cteni ARUCO tagu.
        """
        img = self.image.copy()

        # Tento blok p;jde vyhledem ke statické kameře zavolat jenom jednou.
        # Předělat na service, který v případě, že už bude provedeno pouze vrátí hodnoty.
        # Tím se vše výrazně urychlí
        if self.cells == None:
            img_gray = skimage.color.rgb2gray(img[:,:,:3])
            points = self.get_grid(img_gray)
            # Select corners of grid
            src = np.array([[900,350], [3200, 350], [900, 2650], [3200, 2650]])
            src_new = self.get_new_src_coordinates(src, points)
            dst = np.array([[0, 0], [600, 0], [0, 600], [600, 600]])
            # Transform
            img_transformed = self.transform_image(img, src_new, dst)
            # Detect points once more
            transformed_gray = img_transformed[:,:,0] # Pick red channel only
            points = self.get_grid(transformed_gray)
            self.cells = self.get_cell_centers(points)


        (corners, ids, rejected) = cv2.aruco.detectMarkers(img, self.arucoDict,
            parameters=self.arucoParams)
        # verify *at least* one ArUco marker was detected
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()            
        for (markerCorner, markerID) in zip(corners, ids):
            if markerID == self.arucoId:
                robot_position = self.get_robot_position_px(markerCorner)
                robot_pos_grid = self.get_robot_position_in_grid(cells, robot_position)
        return robot_pos_grid

    def is_move_complete(self, last_state, current_state, move):
        state = current_state - last_state
        if state[2] >= 180:
            state[2] = 360 - state[2]
        if state[2] <= -180:
            state[2] = -360 - state[2]
        if move == "L":
            if state[2] > 85 and state[2] < 95:
                return True
        elif move == "R":
            if state[2] > -95 and state[2] < -85:
                return True            
        elif move == "F":
            if (last_state[2] == 0):
                if state[0] == 1:
                    return True
            elif (last_state[2] == 90):
                if state[1] == 1:
                    return True
            elif (last_state[2] == 180):
                if state[0] == -1:
                    return True
            elif (last_state[2] == 270):
                if state[1] == -1:
                    return True
        elif move == "B":
            if (last_state[2] == 0):
                if state[0] == -1:
                    return True
            elif (last_state[2] == 90):
                if state[1] == -1:
                    return True
            elif (last_state[2] == 180):
                if state[0] == 1:
                    return True
            elif (last_state[2] == 270):
                if state[1] == 1:
                    return True
            pass
        else:
            return True
        return False

    def process_path(self, path):
        while len(path > 0):
            current_move = path.pop(0)
            last_state = self.get_robot_position()
            while current_move is not None:
                current_state = self.get_robot_position()
                # Porovnat current a last state zda doslo ke správnému posunu.
                if self.is_move_complete(last_state=last_state, current_state=current_state, move=current_move):
                    break
                move_msg = TwistStamped()
                if current_move == "L":
                    move_msg.twist.angular.z = -1
                    # SEND MOVE LEFT
                    pass
                elif current_move == "R":
                    move_msg.twist.angular.z = 1
                    # SEND MOVE RIGHT
                    pass
                elif current_move == "F":
                    move_msg.twist.linear.x = 1
                    # SEND MOVE FORWARD
                    pass
                elif current_move == "B":
                    move_msg.twist.linear.x = -1
                    # SEND MOVE BACK
                    pass
                else:
                    # DO NOTHING
                    pass
                self.twist_publisher.Publish(move_msg)
                
            pass

    def run_callback(self):
        if len(self.path) > 0  and self.image is not None and self.trigger is not None:
            self.process_path(self.path.pop(0))
            self.trigger == None
            pass

def main(args=None):
    rclpy.init(args=args)

    controller = LamponeServerRobotController()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()