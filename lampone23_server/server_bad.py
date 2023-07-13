import rclpy
from rclpy.node import Node
from datetime import datetime

import socketserver
import socket
import threading

from std_msgs.msg import String

class ThreadedUDPRequestHandler(Node, socketserver.BaseRequestHandler):

    def __init__(self, request, client_address, server):
        socketserver.BaseRequestHandler.__init__(self, request, client_address, server)
        now = datetime.now() # current date and time
        Node.__init__(self, 'lampone_server_handler_'+ now.strftime("%H%M%S"))
        print("ok")

    def handle(self):
        print("ok")
        data = self.request[0].strip()
        socket = self.request[1]
        print("{} wrote:".format(self.client_address[0]))
        socket.sendto(data.upper(), self.client_address)
        print(data)
        pass

class ThreadedUDPServer(socketserver.ThreadingMixIn, socketserver.UDPServer):
    pass

class LamponeServer(Node):

    def __init__(self):
        super().__init__('lampone_server')
        self.host = "0.0.0.0"
        self.port = 9999
        self.ServerAddress = ("0.0.0.0", 9999)
        # self.publisher_ = self.create_publisher(String, 'topic', 10)
        #timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.i = 0
        """
        server = ThreadedUDPServer((self.host, self.port), ThreadedUDPRequestHandler)
        with server:
            server_thread = threading.Thread(target=server.serve_forever)
            server_thread.daemon = True
            server_thread.start()
            print("Server loop running in thread:", server_thread.name)

            #server.shutdown()
        """
        UDPServerObject = socketserver.ThreadingUDPServer(self.ServerAddress, ThreadedUDPRequestHandler)
        UDPServerObject.serve_forever()

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