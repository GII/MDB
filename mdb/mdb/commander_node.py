import sys
import rclpy
from rclpy.node import Node

from mdb_interfaces.srv import SendCommand

class CommanderNode(Node):
    # TODO: Currently the commands are sent from the terminal. Use the commander node instead.
    """
    This class is responsible for sending commands to the execution nodes.

    It has a client for the 'SendCommand' service.

    Attributes:
        cli (:class:`rclpy.client.Client`): A client for the 'SendCommand' service.


    """


    def __init__(self):
        """
        Constructor for the CommanderNode class.

        Creates a ROS 2 node named 'commander node' and a client for the 'SendCommand' service.
        """
        super().__init__('commander node')
        self.cli = self.create_client(SendCommand, 'send_command')

    def send_request(self):
        """
        Send a request to an execution node.
        """        
        
        self.req.command = str(sys.argv[1])
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    commander = CommanderNode()
    commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()