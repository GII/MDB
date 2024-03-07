import rclpy
from rclpy.node import Node
from core.cognitive_node import CognitiveNode
import random

from std_msgs.msg import Int64

def is_prime(num):
    """
    Check if a number is prime.

    :param int num: The number to check.
    :return: True if the number is prime, False otherwise.
    :rtype: bool
    """
    if num < 2:
        return False
    for n in range(2, int(num/2)+1):
        if num % n == 0:
            return False
    return True

def calculate_n_primes(n):
    """
    Calculate the first n prime numbers.

    :param int n: The number of prime numbers to be calculated.
    :return: A list containing the first n prime numbers.
    :rtype: list[int]
    """
    primes = []
    num = 2
    while len(primes) < n:
        if(is_prime(num)):
            primes.append(num)
        num += 1
    return primes

class ANode(CognitiveNode):
    """
    This is a sample class made for testing the system functionalities.
    It represents a specific type of cognitive node (such as, in the future, the c-nodes or the p-nodes).
    It has a subscription to the topic 'primes' in which publishers ask for the calculation of the first n prime numbers.
    This class calculates the first n primes and also counts the total number of messages received.

    :param name: The name of the node.
    :param msg_count: The initial message count.
    """

    def __init__(self, name='a_node', msg_count=0, class_name = 'cognitive_nodes.a_node.ANode'):
        """
        Initialize the ANode.

        :param name: The name of the node.
        :param msg_count: The initial message count.
        """
        super().__init__(name, class_name)
        self.msg_count = msg_count
        self.subscription = self.create_subscription(
            Int64,
            'primes',
            self.calculate_next_callback,
            10
        )
        self.register_in_LTM({})
        
    def calculate_next_callback(self, msg):
        """
        Calculates the the first n prime numbers.

        :param msg: The message containing n (number of primes to be calculated)
        """        
        n = msg.data
        primes = calculate_n_primes(n)
        self.msg_count += 1
        self.get_logger().info('Node ' + str(self.get_name()) + ' calculated ' + str(n) + ' primes: ' + str(primes) + '. Total msgs received: ' + str(self.msg_count))

    def calculate_activation(self, perception):
        """
        Fake method that simulates the calculation of the activation of the ANode.

        :return: A random float between 0 and 1, representing the activation level.
        :rtype: float
        """
        self.activation = random.random()
        if self.activation_topic:
            self.publish_activation(self.activation)
        return self.activation


def main(args=None):
    rclpy.init(args=args)

    a_node = ANode()

    rclpy.spin(a_node)

    a_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()