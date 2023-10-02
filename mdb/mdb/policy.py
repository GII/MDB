import rclpy
from rclpy.node import Node
from mdb.cognitive_node import CognitiveNode
import random

from std_msgs.msg import Int64
from mdb_interfaces.srv import ExecutePolicy

class Policy(CognitiveNode):


    def __init__(self, name='policy'):

        super().__init__(name, 'mdb.policy.Policy')

        self.register_in_LTM([], [])

        self.execute_policy_service = self.create_service(
            ExecutePolicy,
            'policy/' + str(name) + '/execute',
            self.execute_policy
        )

    def calculate_activation(self): # TODO: Implmement this method
        return random.random()
    
    def execute_policy(self, request, response):
        self.get_logger().info('Executing policy: ' + self.name)
        response.policy = self.name
        return response


def main(args=None):
    rclpy.init(args=args)

    policy = Policy()

    rclpy.spin(policy)

    policy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()