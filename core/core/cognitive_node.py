from abc import ABC, abstractmethod
import yaml
from rclpy.node import Node

from core.service_client import ServiceClient
from core_interfaces.srv import UpdateActivation
from cognitive_node_interfaces.srv import GetActivation, GetInformation, SetActivationTopic
from cognitive_node_interfaces.msg import Activation
from core_interfaces.srv import SendToLTM

class CognitiveNode(ABC, Node):
    """
    A base class for cognitive nodes in the system.

    This class extends the `rclpy.node.Node` class and provides some
    common functionality for cognitive nodes.

    :param name: The name of the node.
    """

    def __init__(self, name, class_name):   # TODO: add state
        """
        Initialize a CognitiveNode.

        :param name: The name of the node.
        """
        super().__init__(name)
        self.name = name
        self.class_name = class_name
        _, _, node_type = self.class_name.rpartition(".")
        self.node_type = node_type

        #Publish node activation when SetActivationTopic is true
        self.publish_activation_topic = self.create_publisher(
            Activation,
            'cognitive_node/' + str(name) + '/activation',
            0
        )
        self.activation_topic = False

        # Update Activations Service for other Cognitive Nodes
        self.update_activation_service = self.create_service(
            UpdateActivation,
            'cognitive_node/' + str(node_type) + '/' + str(name) + '/update_activation',
            self.handle_update_activation
        )

        # N: Get Activation Service
        self.get_activation_service = self.create_service(
            GetActivation,
            'cognitive_node/' + str(name) + '/get_activation',
            self.get_activation_callback
        )
        
        # N: Get Information Service
        self.get_information_service = self.create_service(
            GetInformation,
            'cognitive_node/' + str(name) + '/get_information',
            self.get_information_callback
        )
        self.last_activation = 0.0
        
        # N: Set Activation Topic Service
        self.set_activation_service = self.create_service(
            SetActivationTopic,
            'cognitive_node/' + str(name) + '/set_activation_topic',
            self.set_activation_topic_callback
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
        keys_to_delete = [key for key in node_data.keys() if key.startswith('_') or 'service' in key]
        for key in keys_to_delete:
            del node_data[key]
        del node_data['subscription']
        # del node_data['calculate_activations_service']
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
        :rtype: core_interfaces.srv.SendToLTM_Response
        """
        service_name = 'send_to_LTM'
        send_to_LTM_client = ServiceClient(SendToLTM, service_name)
        ltm_response = send_to_LTM_client.send_request(command=command, name=self.name, node_type=self.node_type, data=data)
        send_to_LTM_client.destroy_node()
        return ltm_response
    
    def handle_update_activation(self, request, response):
        perception = request.perception
        activation = self.calculate_activation(perception)
        response.updated = True
        return response

    def calculate_activation(self, perception):
        """
        Calculate and return the node's activations.

        :param request: The request for calculating activations.
        :type request: core_interfaces.srv.CalculateActivations_Request
        :param response: The response containing the activations.
        :type response: core_interfaces.srv.CalculateActivations_Response
        """
        raise NotImplementedError
    
    def publish_activation(self, activation):
        msg = Activation()
        msg.activation = activation
        self.publish_activation_topic.publish(msg)

    def get_activation_callback(self, request, response): # TODO: implement this method
        self.get_logger().info('Getting node activation...')
        perception = 0 #Only for avoid errors with calculate_activation method
        self.calculate_activation(perception) # TODO: implement logic
        response.activation = self.last_activation
        return response

    def get_information_callback(self, request, response): # TODO: implement this method
        self.get_logger().info('Getting node information...')
        response.current_activation = self.last_activation
        self.get_logger().info('The last activation of the node is: ' + str(response.current_activation))
        return response

    def set_activation_topic_callback(self, request, response):
        activation_topic = request.activation_topic
        self.get_logger().info('Setting activation topic to ' + str(activation_topic) + '...')
        if activation_topic:
            self.activation_topic = True
        else:
            self.activation_topic = False
        response.activation_topic = activation_topic
        return response
    
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
