import rclpy
from rclpy.node import Node

from core.cognitive_node import CognitiveNode
from cognitive_node_interfaces.srv import SetActivation, Evaluate, GetSuccessRate

import random

class UtilityModel(CognitiveNode):
    """
    Utility Model class
    """
    def __init__(self, name='utility_model', class_name = 'cognitive_nodes.utility_model.UtilityModel', **params):
        """
        Constructor of the Utility Model class

        Initializes a Utility Model instance with the given name and registers it in the ltm

        :param name: The name of the Utility Model instance
        :type name: str
        :param class_name: The name of the Utility Model class
        :type class_name: str
        """
        super().__init__(name, class_name, **params)
        self.register_in_LTM({})
        
        # N: Set Activation Service
        self.set_activation_service = self.create_service(
            SetActivation,
            'utility_model/' + str(name) + '/set_activation',
            self.set_activation_callback
        )

        # N: Evaluate Service
        self.evaluate_service = self.create_service(
            Evaluate,
            'utility_model/' + str(name) + '/evaluate',
            self.evaluate_callback
        )

        # N: Get Success Rate Service
        self.get_success_rate_service = self.create_service(
            GetSuccessRate,
            'utility_model/' + str(name) + '/get_success_rate',
            self.get_success_rate_callback
        )

    def set_activation_callback(self, request, response):
        """
        CNodes can modify the utility model's activation

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
    
    def evaluate_callback(self, request, response): # TODO: implement
        """
        Get expected valuation for a given perception

        :param request: The request that contains the perception
        :type request: cognitive_node_interfaces.srv.Evaluate_Request
        :param response: The response that contains tha valuation of the perception
        :type response: cognitive_node_interfaces.srv.Evaluate_Response
        :return: The response that contains the valuation of the perception
        :rtype: cognitive_node_interfaces.srv.Evaluate_Response
        """
        perception = request.perception
        self.get_logger().info('Evaluating for perception ' +str(perception) + '...')
        # TODO: implement logic
        valuation = 3.0
        response.valuation = valuation
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

    def calculate_activation(self, perception = None): #TODO: Implement logic
        """
        Returns the the activation value of the Utility Model

        :param perception: The given percepton
        :type perception: dict
        :return: The activation of the instance
        :rtype: float
        """
        self.activation = random.random()
        if self.activation_topic:
            self.publish_activation(self.activation)
        return self.activation


def main(args=None):
    rclpy.init(args=args)

    utility_model = UtilityModel()

    rclpy.spin(utility_model)

    utility_model.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()