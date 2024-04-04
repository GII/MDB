import numpy
import rclpy
from core.cognitive_node import CognitiveNode
from core.service_client import ServiceClient
from cognitive_node_interfaces.srv import GetActivation
from core.utils import perception_dict_to_msg


class CNode(CognitiveNode):
    """
    CNode class
    It represents a context, that is, a link between nodes that were activated together in the past.
    It is assumed that there is only one element of each type connected to the C-Node.
    """
    def __init__(self, name = 'cnode', class_name = 'cognitive_nodes.cnode.CNode', **params):
        """
        Constructor of the CNode class
        Initializes a CNode with the given name and registers it in the LTM

        :param name: The name of the CNode
        :type name: str
        :param class_name: The name of the CNode class
        :type str
        """
        super().__init__(name, class_name, **params)
        self.register_in_LTM({})

    def calculate_activation(self, perception=None):
        """
        Calculate the new activation value by multiplying the activation values of its previous neighbors.
        By default, with percerception = None, it will multiply the last activations of its neighbots, but 
        it's possible to use an arbitrary perception, that will propagate to the neighbors, calculating the 
        final activation of the CNode for that perception.

        :param perception: Arbitraty perception 
        :type perception: dict
        :return: The activation of the CNode
        :rtype: float
        """
        node_activations = []
        neighbors_name = [neighbor["name"] for neighbor in self.neighbors if neighbor["node_type"] != "Policy"]
        for name in neighbors_name:
            service_name = 'cognitive_node/' + str(name) + '/get_activation'
            activation_client = ServiceClient(GetActivation, service_name)
            perception = perception_dict_to_msg(perception)
            activation = activation_client.send_request(perception = perception)
            activation_client.destroy_node()
            node_activations.append(activation)

        activation_list = numpy.prod(node_activations)
        self.activation = numpy.max(activation_list)
        #TODO: Selection of the perception that have the max CNode or PNode activation (if it exists), as in the old MDB

        self.get_logger().info(self.node_type + " activation for " + self.name + " = " + str(self.activation))
        if self.activation_topic:
            self.publish_activation(self.activation)
            
        return self.activation
