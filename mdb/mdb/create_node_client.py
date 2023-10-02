import sys

import rclpy
from rclpy.node import Node

from mdb_interfaces.srv import CreateNode


class CreateNodeClient(Node):

    def __init__(self, service_name):

        super().__init__('create_node_client')
        self.cli = self.create_client(CreateNode, service_name)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = CreateNode.Request()

    def send_request(self, name, class_name):
        self.req.name = name
        self.req.class_name = class_name
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

# TODO: Check if it's necessary a main method
def main():
    rclpy.init()
    minimal_client = CreateNodeClient()
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()