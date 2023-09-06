import sys
import rclpy
from rclpy.node import Node

from mdb_interfaces.srv import SendCommand

class CommanderNode(Node):

    def __init__(self):
        super().__init__('commander node')
        self.cli = self.create_client(SendCommand, 'send_command')

    def send_request(self):
        self.req.command = str(sys.argv[1])
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    commander = CommanderNode()
    commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()