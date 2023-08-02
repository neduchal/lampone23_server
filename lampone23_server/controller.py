import rclpy
from rclpy.node import Node
# dependencies rclpy image_transport cv_bridge sensor_msgs std_msgs opencv2
import cv2
from cv_bridge import CvBridge 
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, String # For trigger message

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
        pass

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
                    move_msg.twist.angular.z = -1
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