import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

class LamponeServerProcesser(Node):

    pass


def main(args=None):
    rclpy.init(args=args)

    processer = LamponeServerProcesser()

    rclpy.spin(processer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    processer .destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()