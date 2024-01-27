import rclpy
from rclpy.node import Node

class ServiceClient(Node):
    """
    A generic service client class.
    """

    def __init__(self, service_type, service_name):
        """
        Constructor for the ServiceClient class.

        :param service_type: The type of the ROS 2 service.
        :type service_type: type
        :param service_name: The name of the ROS 2 service.
        :type service_name: str
        """        
        client_name = service_name.replace("/", "_") + '_client'
        super().__init__(client_name)
        self.cli = self.create_client(service_type, service_name)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = service_type.Request()

    def send_request(self, **kwargs):
        """
        Send a request to the service.

        :param kwargs: Keyword arguments representing the request parameters.
        :type kwargs: dict
        :return: The response from the service.
        :rtype: type
        """        
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
