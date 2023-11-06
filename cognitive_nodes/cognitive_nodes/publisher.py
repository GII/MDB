import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64

import random

class MinimalPublisher(Node):
    """
    This is a sample class made for testing the system functionalities.
    It publishes messages asking other nodes for a random number (between 5 and 15) of prime numbers,
    and a random number (between 5 and 15) of Fibonacci numbers.
    """

    def __init__(self):
        """
        Constructor for the MinimalPublisher class.
        """
        super().__init__('minimal_publisher')
        self.primes_publisher_ = self.create_publisher(Int64, 'primes', 10)
        self.fibonacci_publisher_ = self.create_publisher(Int64, 'fibonacci', 10)
        self.primes_timer_period = 0.5  # seconds
        self.fibonacci_timer_period = 0.7  # seconds
        self.primes_timer = self.create_timer(self.primes_timer_period, self.primes_timer_callback)
        self.fibonacci_timer = self.create_timer(self.fibonacci_timer_period, self.fibonacci_timer_callback)
        self.i = 0

    def primes_timer_callback(self):
        """
        Callback for the primes timer. Requests a random number of prime numbers between 5 and 15.
        """
        n = random.randint(5, 15)
        msg = Int64()
        msg.data = n
        self.primes_publisher_.publish(msg)
        self.i += 1
        self.get_logger().info("Asking for " + str(msg.data) + " prime numbers. Call " + str(self.i) + ".")

    def fibonacci_timer_callback(self):
        """
        Callback for the Fibonacci timer. Requests a random number of Fibonacci numbers between 5 and 15.
        """
        n = random.randint(5, 15)  # Generamos un número aleatorio entre 5 y 15
        msg = Int64()
        msg.data = n
        self.fibonacci_publisher_.publish(msg)
        self.i += 1
        self.get_logger().info("Asking for " + str(msg.data) + " Fibonacci numbers. Call " + str(self.i) + ".")


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
