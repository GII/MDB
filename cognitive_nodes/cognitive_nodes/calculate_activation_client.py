import sys

import rclpy
from rclpy.node import Node

from core_interfaces.srv import CalculateActivation


class CalculateActivationClient(Node):

    def __init__(self, service_name):

        super().__init__('create_node_client')
        self.cli = self.create_client(CalculateActivation, service_name)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = CalculateActivation.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

# TODO: Check if it's necessary a main method
def main():
    rclpy.init()
    minimal_client = CalculateActivationClient()
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()