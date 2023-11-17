import rclpy
from rclpy.node import Node

from core.cognitive_node import CognitiveNode
from cognitive_node_interfaces.srv import SetActivation, Predict, GetSuccessRate, IsCompatible

import random

class WorldModel(CognitiveNode):

    def __init__(self, name='world_model', class_name = 'cognitive_nodes.world_model.WorldModel'):

        super().__init__(name, class_name)
        self.register_in_LTM({})

        # N: Set Activation Service
        self.set_activation_service = self.create_service(
            SetActivation,
            'world_model/' + str(name) + '/set_activation',
            self.set_activation_callback
        )

        # N: Predict Service
        self.predict_service = self.create_service(
            Predict,
            'world_model/' + str(name) + '/predict',
            self.predict_callback
        )

        # N: Get Success Rate Service
        self.get_success_rate_service = self.create_service(
            GetSuccessRate,
            'world_model/' + str(name) + '/get_success_rate',
            self.get_success_rate_callback
        )

        # N: Is Compatible Service
        self.is_compatible_service = self.create_service(
            IsCompatible,
            'world_model/' + str(name) + '/is_compatible',
            self.is_compatible_callback
        )

    def set_activation_callback(self, request, response): # TODO: implement
        self.get_logger().info('Setting activation..')
        # TODO: implement logic
        response.set = True
        return response
    
    def predict_callback(self, request, response): # TODO: implement
        timestamp = request.timestamp
        policy = request.policy
        self.get_logger().info('Predicting for policy ' +str(policy) + ' at ' + str(timestamp) + '...')
        # TODO: implement logic
        perception = [0.35, 0.36]
        response.perception = perception
        return response
    
    def get_success_rate_callback(self, request, response): # TODO: implement
        self.get_logger().info('Getting success rate..')
        # TODO: implement logic
        response.success_rate = 0.5
        return response
    
    def is_compatible_callback(self, request, response): # TODO: implement
        self.get_logger().info('Checking if compatible..')
        # TODO: implement logic
        response.compatible = True
        return response

    def calculate_activation(self, world_model):
        return random.random()


def main(args=None):
    rclpy.init(args=args)

    world_model = WorldModel()

    rclpy.spin(world_model)

    world_model.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()