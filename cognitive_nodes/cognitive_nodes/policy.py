import rclpy
from rclpy.node import Node
from core.cognitive_node import CognitiveNode
import random

from std_msgs.msg import Int64
from cognitive_node_interfaces.srv import SetActivation, Execute

class Policy(CognitiveNode):
    """
    Policy class.
    """

    def __init__(self, name='policy', class_name='core.policy.Policy'):
        """
        Constructor for the Policy class.

        Initializes a policy with the given name and registers it in the LTM.
        It also creates a service for executing the policy.

        :param name: The name of the policy.
        :type name: str
        """

        super().__init__(name, 'core.policy.Policy')

        self.register_in_LTM([], [])

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

    def update_activation_from_old_mdb(self, **kwargs):
        """
        Calculate the new activation value.

        This activation value is the maximum of the connected c-nodes.
        """
        cnodes = [node for node in self.neighbors if node.type == "CNode"]
        if cnodes:
            cnode = max(cnodes, key=attrgetter("activation"))
            self.perception = cnode.perception
            self.activation = cnode.activation
        else:
            self.perception = []
            self.activation = 0.0
        self.get_logger().info(self.type + " activation for " + self.ident + " = " + str(self.activation))
        self.publish()     

    def execute_from_old_mdb(self):
        """Run the policy."""
        self.get_logger().info()"Executing policy " + self.ident)
        self.data_publisher.publish(self.ident)           

    def calculate_activation(self, perception): # TODO: Implmement this method
        """
        Calculate the activation level of the policy: a random float between 0 and 1.

        Mock method that pretends to calculate the activation level of the policy node.

        :return: The activation level, a random float between 0 and 1.
        :rtype: float
        """
        return random.random()
    
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
    
    def set_activation_callback(self, request, response): # TODO: implement
        activation = request.activation
        self.get_logger().info('Setting activation ' + str(activation) + '...')
        # TODO: implement logic
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