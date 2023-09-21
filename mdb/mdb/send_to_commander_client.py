import sys

import rclpy
from rclpy.node import Node

from mdb_interfaces.srv import SendToCommander


class SendToCommanderClient(Node):
    """
    This class represents a client that sends requests to the commander.

    Attributes:
        cli (:class:`rclpy.client.Client`): A client for the 'SendToCommander' service.
        req (:class:`mdb_interfaces.srv.SendToCommander.Request`): A request for the service.

    """

    def __init__(self):
        """
        Constructor for the SendToCommanderClient class.

        Creates a ROS 2 node named 'send_to_commander_client' and a client for the 'SendToCommander' service.
        """        

        super().__init__('send_to_commander_client')
        self.cli = self.create_client(SendToCommander, 'send_to_commander')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SendToCommander.Request()

    def send_request(self, command, name, type, data):
        """
        Send a request to the commander node.

        :param command: The command for the commander node.
        :type command: str
        :param name: The name of the cognitive node.
        :type name: str
        :param type: The type of the cognitive node.
        :type type: str
        :param data: Optional data to initialize the cognitive node.
        :type data: str
        :return: The response from the execution node.
        :rtype: mdb_interfaces.srv.SendToCommander.Response
        """
        self.req.command = command
        self.req.name = name
        self.req.type = type
        self.req.data = data
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

# TODO: Check if it's necessary a main method
def main():
    rclpy.init()
    minimal_client = SendToCommanderClient()
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()