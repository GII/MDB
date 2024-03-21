import rclpy
from rclpy.node import Node
from core.cognitive_node import CognitiveNode
import random
import numpy

from std_msgs.msg import Int64
from core.service_client import ServiceClient
from cognitive_node_interfaces.srv import SetActivation, Execute
from cognitive_node_interfaces.srv import GetActivation

class Policy(CognitiveNode):
    """
    Policy class.
    """

    def __init__(self, name='policy', class_name='cognitive_nodes.policy.Policy', **params):
        """
        Constructor for the Policy class.

        Initializes a policy with the given name and registers it in the LTM.
        It also creates a service for executing the policy.

        :param name: The name of the policy.
        :type name: str
        """

        super().__init__(name, 'cognitive_nodes.policy.Policy', **params)

        self.register_in_LTM({})

        self.set_activation_service = self.create_service(
            SetActivation,
            'policy/' + str(name) + '/set_activation',
            self.set_activation_callback
        )

        self.execute_service = self.create_service(
            Execute,
            'policy/' + str(name) + '/execute',
            self.execute_callback
        )         

    def calculate_activation(self, perception):
        """
        Calculate the activation level of the policy: a random float between 0 and 1.

        Mock method that pretends to calculate the activation level of the policy node.

        :return: The activation level, a random float between 0 and 1.
        :rtype: float
        """
        cnodes = [neighbor["name"] for neighbor in self.neighbors if neighbor["node_type"] == "CNode"]
        if cnodes:
            cnode_activations = []
            for cnode in cnodes:
                service_name = 'cognitive_node/' + str(cnode) + '/get_activation'
                activation_client = ServiceClient(GetActivation, service_name)
                perception = self.perception_dict_to_msg(perception = None)
                activation = activation_client.send_request(perception = perception)
                activation_client.destroy_node()
                cnode_activations.append(activation)
                self.activation = numpy.max(cnode_activations)
        else:
            self.activation = 0.0
        
        self.get_logger().info(self.node_type + " activation for " + self.name + " = " + str(self.activation))
        if self.activation_topic:
            self.publish_activation(self.activation)
        return self.activation
    
    def execute_callback(self, request, response): # TODO: implement

        """
        Mock method that pretends to execute the policy.
        It logs the execution and returns the policy name in the response.

        :param request: The request to execute the policy.
        :type request: core_interfaces.srv.ExecutePolicy_Request
        :param response: The response indicating the executed policy.
        :type response: core_interfaces.srv.ExecutePolicy_Response
        :return: The response with the executed policy name.
        :rtype: core_interfaces.srv.ExecutePolicy_Response
        """
        self.get_logger().info('Executing policy: ' + self.name + '...')
        # TODO: implement logic
        response.policy = self.name
        return response
    
    def set_activation_callback(self, request, response):
        activation = request.activation
        self.get_logger().info('Setting activation ' + str(activation) + '...')
        self.activation = activation
        response.set = True
        return response


def main(args=None):
    rclpy.init(args=args)

    policy = Policy()

    rclpy.spin(policy)

    policy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()