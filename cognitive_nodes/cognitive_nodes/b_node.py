import random
import rclpy
from rclpy.node import Node
from core.cognitive_node import CognitiveNode

from std_msgs.msg import Int64

def calculate_fibonacci(n):
    """
    Calculate the first n Fibonacci numbers.

    :param int n: The number of Fibonacci numbers to be calculated.
    :return: A list containing the first n Fibonacci numbers.
    :rtype: list[int]
    """
    fibonacci = []
    a, b = 0, 1
    while len(fibonacci) < n:
        fibonacci.append(a)
        a, b = b, a + b
    return fibonacci

class BNode(CognitiveNode):
    """
    This is a sample class made for testing the system functionalities.
    It represents a specific type of cognitive node (such as, in the future, the c-nodes or the p-nodes).
    It has a subscription to the topic 'fibonacci' in which publishers ask for the calculation of the first n Fibonacci numbers.
    This class calculates the first n Fibonacci numbers and also counts the total number of messages received.

    :param name: The name of the node.
    :param msg_count: The initial message count.
    """

    def __init__(self, name='b_node', msg_count=0, class_name = 'cognitive_nodes.b_node.BNode'):
        """
        Initialize the BNode.

        :param name: The name of the node.
        :param msg_count: The initial message count.
        """
        super().__init__(name, class_name)
        self.msg_count = msg_count
        self.subscription = self.create_subscription(
            Int64,
            'fibonacci',
            self.calculate_next_callback,
            10
        )
        self.register_in_LTM({})

    def calculate_next_callback(self, msg):
        """
        Calculates the the first n Fibonacci numbers.

        :param msg: The message containing n (number of Fibonacci numbers to be calculated)
        """ 
        n = msg.data
        fibonacci = calculate_fibonacci(n)
        self.msg_count += 1
        self.get_logger().info('Node ' + str(self.get_name()) + ' calculated ' + str(n) + ' Fibonacci numbers: ' + str(fibonacci) + '. Total msgs received: ' + str(self.msg_count))
    
    def calculate_activation(self, perception):
        """
        Fake method that simulates the calculation of the activation of the BNode.

        :return: A random float between 0 and 1, representing the activation level.
        :rtype: float
        """
        return random.random()

def main(args=None):
    rclpy.init(args=args)

    b_node = BNode()

    rclpy.spin(b_node)

    b_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()