from abc import ABC, abstractmethod
import yaml
from rclpy.node import Node

from mdb.send_to_ltm_client import SendToLTMClient
from mdb_interfaces.srv import CalculateActivation

class CognitiveNode(ABC, Node):
    """
    A base class for cognitive nodes in the system.

    This class extends the `rclpy.node.Node` class and provides some
    common functionality for cognitive nodes.

    :param name: The name of the node.
    """

    def __init__(self, name, node_type):   # TODO: add state
        """
        Initialize a CognitiveNode.

        :param name: The name of the node.
        """
        super().__init__(name)
        self.name = name
        self.node_type = node_type
                        
        # Calculate Activations Service for other Cognitive Nodes
        self.calculate_activations_service = self.create_service(
            CalculateActivation,
            'cognitive_node/' +str(node_type) + '/' + str(name) + '/calculate_activation',
            self.handle_calculate_activation
        )

    def get_data(self):
        """
        Get the data associated with the node.

        This method returns a dictionary containing the attributes of
        the node, excluding private attributes and the 'subscription'
        attribute found in the ANode class.

        :return: A dictionary with node data.
        :rtype: dict
        """
        node_data = self.__dict__.copy()
        keys_to_delete = [key for key in node_data.keys() if key.startswith('_')]
        for key in keys_to_delete:
            del node_data[key]
        del node_data['subscription']
        del node_data['calculate_activations_service']
        return node_data
    
    def register_in_LTM(self, subscribed, publishing):
        
        data_dic = {
            'subscribed': subscribed,
            'publishing': publishing
        }

        data = yaml.dump(data_dic)

        self.send_request_to_LTM('register', data)
    
    def suscribe(self, topic):
        self.send_request_to_LTM('subscribe', topic)

    def puslish(self, topic):
        self.send_request_to_LTM('publish', topic)
   
    def send_request_to_LTM(self, command, data):
        """
        Send a request to the LTM.

        :param command: The command to send.
        :type command: str
        :param name: The name of the node.
        :type name: str
        :param type: The type of the node.
        :type type: str
        :param data: Optional data.
        :type data: str
        :return: The response from the LTM.
        :rtype: mdb_interfaces.srv.SendToLTM_Response
        """
        send_to_LTM_client = SendToLTMClient()
        ltm_response = send_to_LTM_client.send_request(command, self.name, self.node_type, data)
        send_to_LTM_client.destroy_node()
        return ltm_response
    
    def handle_calculate_activation(self, request, response):
        response.activation = self.calculate_activation()
        return response

    @abstractmethod
    def calculate_activation(self):
        """
        Calculate and return the node's activations.

        :param request: The request for calculating activations.
        :type request: mdb_interfaces.srv.CalculateActivations_Request
        :param response: The response containing the activations.
        :type response: mdb_interfaces.srv.CalculateActivations_Response
        """
        pass
    


    def __str__(self):
        """
        Returns a YAML representation of the node's data.

        :return: YAML representation of the node's data.
        :rtype: str
        """
        data = self.get_data()
        return yaml.dump(data, default_flow_style=False)

def main(args=None):
    pass

if __name__ == '__main__':
    main()
