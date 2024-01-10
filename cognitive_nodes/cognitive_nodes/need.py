import rclpy
from rclpy.node import Node

from core.cognitive_node import CognitiveNode
from cognitive_node_interfaces.srv import SetActivation, IsSatisfied

import random

class Need(CognitiveNode):

    def __init__(self, name='need', class_name = 'cognitive_nodes.need.Need', **params):

        super().__init__(name, class_name, **params)
        self.register_in_LTM({})
        
        # N: Set Activation Service
        self.set_activation_service = self.create_service(
            SetActivation,
            'need/' + str(name) + '/set_activation',
            self.set_activation_callback
        )

        # N: Is Satisfied Service
        self.is_satisfied_service = self.create_service(
            IsSatisfied,
            'need/' + str(name) + '/is_satisfied',
            self.is_satisfied_callback
        )

    def set_activation_callback(self, request, response):
        activation = request.activation
        self.get_logger().info('Setting activation ' + str(activation) + '...')
        # TODO: implement logic
        response.set = True
        return response
    
    def is_satisfied_callback(self, request, response):
        self.get_logger().info('Checking if is satisfied..')
        # TODO: implement logic
        response.satisfied = True
        return response


    def calculate_activation(self, need):
        return random.random()


def main(args=None):
    rclpy.init(args=args)

    need = Need()

    rclpy.spin(need)

    need.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()