import rclpy
from rclpy.node import Node

from core.cognitive_node import CognitiveNode
from cognitive_node_interfaces.srv import SetActivation, Predict, GetSuccessRate, IsCompatible

import random

class WorldModel(CognitiveNode):
    """
    World Model class
    """
    def __init__(self, name='world_model', class_name = 'cognitive_nodes.world_model.WorldModel', **params):
        """
        Constructor of the World Model class

        Initializes a World Model instance with the given name and registers it in the ltm

        :param name: The name of the World Model instance
        :type name: str
        :param class_name: The name of the World Model class
        :type class_name: str
        """
        super().__init__(name, class_name, **params)
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

    def set_activation_callback(self, request, response):
        """
        Some processes can modify the activation of a World Model

        :param request: The request that contains the new activation value
        :type request: cognitive_node_interfaces.srv.SetActivation_Request
        :param response: The response indicating if the activation was set
        :type response: cognitive_node_interfaces.srv.SetActivation_Response
        :return: The response indicating if the activation was set
        :rtype: cognitive_node_interfaces.srv.SetActivation_Response
        """
        activation = request.activation
        self.get_logger().info('Setting activation ' + str(activation) + '...')
        self.activation = activation
        response.set = True
        return response
    
    def predict_callback(self, request, response): # TODO: implement
        """
        Get predicted perception values for the last perceptions not newer than a given
        timestamp and for a given policy

        :param request: The request that contains the timestamp and the policy
        :type request: cognitive_node_interfaces.srv.Predict_Request
        :param response: The response that included the obtained perception
        :type response: cognitive_node_interfaces.srv.Predict_Response
        :return: The response that included the obtained perception
        :rtype: cognitive_node_interfaces.srv.Predict_Response
        """
        timestamp = request.timestamp
        policy = request.policy
        self.get_logger().info('Predicting for policy ' +str(policy) + ' at ' + str(timestamp) + '...')
        # TODO: implement logic
        perception = [0.35, 0.36]
        response.perception = perception
        return response
    
    def get_success_rate_callback(self, request, response): # TODO: implement
        """
        Get a prediction success rate based on a historic of previous predictions

        :param request: Empty request
        :type request: cognitive_node_interfaces.srv.GetSuccessRate_Request
        :param response: The response that contains the predicted success rate
        :type response: cognitive_node_interfaces.srv.GetSuccessRate_Response
        :return: The response that contains the predicted success rate
        :rtype: cognitive_node_interfaces.srv.GetSuccessRate_Response
        """
        self.get_logger().info('Getting success rate..')
        # TODO: implement logic
        response.success_rate = 0.5
        return response
    
    def is_compatible_callback(self, request, response): # TODO: implement
        """
        Check if the World Model is compatible with the current avaliable perceptions

        :param request: The request that contains the current avaliable perceptions
        :type request: cognitive_node_interfaces.srv.IsCompatible_Request
        :param response: The response indicating if the World Model is compatible or not
        :type response: cognitive_node_interfaces.srv.IsCompatible_Response
        :return: The response indicating if the World Model is compatible or not
        :rtype: cognitive_node_interfaces.srv.IsCompatible_Response
        """
        self.get_logger().info('Checking if compatible..')
        # TODO: implement logic
        response.compatible = True
        return response

    def calculate_activation(self, perception = None):
        """
        Returns the the activation value of the World Model

        :param perception: Perception does not influence the activation 
        :type perception: dict
        :return: The activation of the instance
        :rtype: float
        """

        if self.activation_topic:
            self.publish_activation(self.activation)
        return self.activation


def main(args=None):
    rclpy.init(args=args)

    world_model = WorldModel()

    rclpy.spin(world_model)

    world_model.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()