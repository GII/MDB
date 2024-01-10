import rclpy
from rclpy.node import Node

from core.cognitive_node import CognitiveNode
from cognitive_node_interfaces.srv import SetActivation, Evaluate, GetSuccessRate

import random

class Drive(CognitiveNode):

    def __init__(self, name='drive', class_name = 'cognitive_nodes.drive.Drive', **params):

        super().__init__(name, class_name, **params)
        self.register_in_LTM({})

        # N: Set Activation Service
        self.set_activation_service = self.create_service(
            SetActivation,
            'drive/' + str(name) + '/set_activation',
            self.set_activation_callback
        )

        # N: Evaluate Service
        self.evaluate_service = self.create_service(
            Evaluate,
            'drive/' + str(name) + '/evaluate',
            self.evaluate_callback
        )

        # N: Get Success Rate Service
        self.get_success_rate_service = self.create_service(
            GetSuccessRate,
            'drive/' + str(name) + '/get_success_rate',
            self.get_success_rate_callback
        )

    def set_activation_callback(self, request, response): # TODO: implement
        activation = request.activation
        self.get_logger().info('Setting activation ' + str(activation) + '...')
        # TODO: implement logic
        response.set = True
        return response
    
    def evaluate_callback(self, request, response): # TODO: implement
        perception = request.perception
        self.get_logger().info('Evaluating for perception ' +str(perception) + '...')
        # TODO: implement logic
        valuation = 3.0
        response.valuation = valuation
        return response
    
    def get_success_rate_callback(self, request, response): # TODO: implement
        self.get_logger().info('Getting success rate..')
        # TODO: implement logic
        response.success_rate = 0.5
        return response

    def calculate_activation(self, drive):
        return random.random()


def main(args=None):
    rclpy.init(args=args)

    drive = Drive()

    rclpy.spin(drive)

    drive.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()