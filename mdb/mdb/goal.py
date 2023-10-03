import rclpy
from rclpy.node import Node

from mdb.cognitive_node import CognitiveNode
from cognitive_node_interfaces.srv import SetActivation, IsReached

import random

class Goal(CognitiveNode):

    def __init__(self, name='goal', class_name = 'mdb.goal.Goal'):

        super().__init__(name, class_name)

        # N: Set Activation Service
        self.set_activation_service = self.create_service(
            SetActivation,
            'goal/' + str(name) + '/set_activation',
            self.set_activation_callback
        )

        # N: Is Reached Service
        self.is_reached_service = self.create_service(
            IsReached,
            'goal/' + str(name) + '/is_reached',
            self.is_reached_callback
        )

    def set_activation_callback(self, request, response):
        activation = request.activation
        self.get_logger().info('Setting activation ' + str(activation) + '...')
        # TODO: implement logic
        response.set = True
        return response
    
    def is_reached_callback(self, request, response):
        self.get_logger().info('Checking if is reached..')
        # TODO: implement logic
        response.reached = True
        return response

    def calculate_activation(self, goal):
        return random.random()


def main(args=None):
    rclpy.init(args=args)

    goal = Goal()

    rclpy.spin(goal)

    goal.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()