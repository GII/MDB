import rclpy
from rclpy.node import Node

from core.cognitive_node import CognitiveNode
from cognitive_node_interfaces.srv import SetActivation, IsSatisfied, SetWeight

import random

class Need(CognitiveNode):
    """"
    Need Class
    """
    def __init__(self, name='need', class_name = 'cognitive_nodes.need.Need', **params):
        """
        Constructor of the Need class

        Initializes a Need instance with the given name and registers it in the ltm

        :param name: The name of the Need instance
        :type name: str
        :param class_name: The name of the Need class
        :type class_name: str
        """
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

        self.set_weight_service = self.create_service(
            SetWeight,
            'need/' + str(name) + '/set_weight',
            self.set_weight_service
        )

        self.weight = 1.0

    def set_weight_callback(self, request, response):
        """
        Set the weight of the need

        :param request: Request that contains the new weight of the need
        :type request: cognitive_node_interfaces.srv.SetWeight_Request
        :param response: The response indicating if the weight was set
        :type response: cognitive_node_interfaces.srv.SetWeight_Response
        :return: The response indicating if the weight was set
        :rtype: cognitive_node_interfaces.srv.SetWeight_Response
        """
        weight_value = request.weight
        self.weight = weight_value
        self.get_logger().info('Setting weight value ' + str(weight_value) + '...')
        response.set = True
        return response

    def set_activation_callback(self, request, response):
        """
        Purposes can modify the need's activation

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
    
    def is_satisfied_callback(self, request, response):
        """
        Check if the need had been satisfied

        :param request: Empty request
        :type request: cognitive_node_interfaces.srv.IsSatisfied_Request
        :param response: Response that indicates if the need is satisfied or not
        :type response: cognitive_node_interfaces.srv.IsSatisfied_Response
        :return: Response that indicates if the need is satisfied or not
        :rtype: cognitive_node_interfaces.srv.IsSatisfied_Response
        """
        self.get_logger().info('Checking if is satisfied..')
        # TODO: implement logic
        response.satisfied = True
        return response


    def calculate_activation(self, perception = None): #TODO: Implement logic
        """
        Returns the the activation value of the need

        :param perception: The given perception
        :type perception: dict
        :return: The activation of the need
        :rtype: float
        """
        self.activation = self.weight * random.random()
        if self.activation_topic:
            self.publish_activation(self.activation)
        return self.activation

def main(args=None):
    rclpy.init(args=args)

    need = Need()

    rclpy.spin(need)

    need.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()