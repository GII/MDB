import sys

import rclpy
from rclpy.node import Node

from core_interfaces.srv import SaveNode


class SaveNodeClient(Node):

    def __init__(self, service_name):

        super().__init__('save_node_client')
        self.cli = self.create_client(SaveNode, service_name)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SaveNode.Request()

    def send_request(self, name):
        self.req.name = name
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

# TODO: Check if it's necessary a main method
def main():
    rclpy.init()
    minimal_client = SaveNodeClient()
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()