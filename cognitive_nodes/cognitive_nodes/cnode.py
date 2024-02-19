import numpy
import rclpy
from core.cognitive_node import CognitiveNode
from core.service_client import ServiceClient
from cognitive_node_interfaces.srv import GetActivation


class CNode(CognitiveNode):
    """
    It represents a context, that is, a link between nodes that were activated together in the past.

    It is assumed that there is only one element of each type connected to the C-Node.
    """
    def __init__(self, name = 'cnode', class_name = 'cognitive_nodes.cnode.CNode', **params):
        super().__init__(name, class_name, **params)
        self.register_in_LTM({})

    def calculate_activation(self, perception=None):
        """Calculate the new activation value."""
        node_activations = []
        neighbors_name = [neighbor["name"] for neighbor in self.neighbors if neighbor["node_type"] != "Policy"]
        for name in neighbors_name:
            service_name = 'cognitive_node/' + str(name) + '/get_activation'
            activation_client = ServiceClient(GetActivation, service_name)
            # We don't know the format of the perception variable. It could contain only one perception or
            # several ones
            activation = activation_client.send_request(perception = perception)
            activation_client.destroy_node()
            node_activations.append(activation)

        activation_list = numpy.prod(node_activations)
        self.last_activation = numpy.max(activation_list)
        #TODO: Selection of the perception that have the max CNode or PNode activation (if it exists), as in the old MDB

        self.get_logger().info(self.node_type + " activation for " + self.name + " = " + str(self.last_activation))


    def update_activation_old_mdb(self, **kwargs):
        """
        Calculate the new activation value.

        This activation value is the product of the activation value of the connected nodes, excluding the policy.
        It is assumed that all the neighbors have the same list of perceptions but, probably, it should
        be checked (although this would have a huge performance penalty).
        """
        pnode = self.p_node
        activation_list = numpy.prod([node.activation for node in self.neighbors if node.type != "Policy"], axis=0)
        self.activation = numpy.max(activation_list)
        if self.activation > self.threshold:
            self.perception = pnode.perception[numpy.argmax(activation_list)]
        else:
            # Even if there is not C-node activation, we want to know the perception that leaded to the hightest
            # P-node activation, in order to add a point to the P-node / create a new P-node and C-node
            # when a random policy is executed and that execution would satisfy a goal in that goal was activated.
            # This is an interin solution, see __add_point()...
            if numpy.max(pnode.activation) > self.threshold:
                self.perception = pnode.perception[numpy.argmax(pnode.activation)]
            else:
                self.perception = []
        # rospy.logdebug(self.type + " activation for " + self.ident + " = " + str(self.activation))
        self.publish()