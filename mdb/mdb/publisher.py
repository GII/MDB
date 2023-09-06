import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Int64, 'primes', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Int64()
        msg.data = 10
        self.publisher_.publish(msg)
        self.i += 1
        self.get_logger().info("Asking for " + str(msg.data) + " prime numbers. Call " + str(self.i) + ".")


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()