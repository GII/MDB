import rclpy
from rclpy.node import Node

class ServiceClient(Node):

    def __init__(self, service_type, service_name):
        client_name = service_name.replace("/", "_") + '_client'
        super().__init__(client_name)
        self.cli = self.create_client(service_type, service_name)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = service_type.Request()

    def send_request(self, **kwargs):
        for key, value in kwargs.items():
            setattr(self.req, key, value)
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main():
    rclpy.init()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
