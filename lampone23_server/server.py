import rclpy
from rclpy.node import Node

import socketserver
import socket

from std_msgs.msg import String

class LamponeServer(Node):

    def __init__(self):
        super().__init__('lampone_server')
        self.host = "0.0.0.0"
        self.port = 9999
        self.server_address = (self.host, self.port)
        self.publisher_ = self.create_publisher(String, '/lampone23/path', 10)
        #timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(self.server_address)

        while True:
            data, addr = self.sock.recvfrom(2048) # buffer size is 1024 bytes
            print(addr)
            print("received message: %s" % data)
            self.sock.sendto(b"MESSAGE RECEIVED", addr)
            msg = String()
            msg.data = data.decode('utf-8')
            self.publisher_.publish(msg)
            

    def onShutdown(self):
        self.sock.close()

    """
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
    """


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = LamponeServer()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()