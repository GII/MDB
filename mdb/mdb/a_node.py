import rclpy
from rclpy.node import Node
from mdb.cognitive_node import CognitiveNode

from std_msgs.msg import Int64

    
def is_prime(num):
    if num < 2:
        return False
    for n in range(2, int(num/2)+1):
        if num % n == 0:
            return False
    return True

def calculate_n_primes(n):
    primes = []
    num = 2
    while len(primes) < n:
        if(is_prime(num)):
            primes.append(num)
        num += 1
    return primes

class ANode(CognitiveNode):

    def __init__(self, name='subscriber', msg_count=0):
        super().__init__(name)
        self.msg_count = msg_count

        # cuidado, referencia!
        self.subscription = self.create_subscription(
            Int64,
            'primes',
            self.calculate_next_callback,
            10
        )
    
    def get_state(self):
        # state = self.__dict__.copy() 
        # del state['_Node__executor_weakref']
        # del state['_Node__node']
        state = {}
        state['msg_count'] = self.msg_count
        # state = {'msg_count': self.msg_count}
        return state
    
    def set_state(self, new_state):
        self.msg_count = new_state['msg_count']

    def calculate_next_callback(self, msg):
        n = msg.data
        primes = calculate_n_primes(n)
        self.msg_count += 1
        self.get_logger().info('Node ' + str(self.get_name()) + ' calculated ' + str(n) + ' primes: ' + str(primes) + '. Total msgs received: ' + str(self.msg_count))


def main(args=None):
    rclpy.init(args=args)

    a_node = ANode()

    rclpy.spin(a_node)

    a_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()