from abc import ABC, abstractmethod
import yaml
from rclpy.node import Node

from core.service_client import ServiceClient
from core_interfaces.srv import AddNodeToLTM, DeleteNodeFromLTM
from cognitive_node_interfaces.srv import GetActivation, GetInformation, SetActivationTopic
from cognitive_node_interfaces.msg import Activation

class CognitiveNode(ABC, Node):
    """
    A base class for cognitive nodes in the system.

    This class extends the `rclpy.node.Node` class and provides some
    common functionality for cognitive nodes.

    :param name: The name of the node.
    """

    def __init__(self, name, class_name, **params):
        """
        Initialize a CognitiveNode.

        :param name: The name of the node.
        """
        super().__init__(name)
        self.name = name
        self.class_name = class_name
        _, _, node_type = self.class_name.rpartition(".")
        self.node_type = node_type

        self.perception = None
        self.activation = 0.0

        # self.threshold = threshold
        self.neighbors = [] # List of dics, like [{"name": "pnode1", "node_type": "PNode"}, {"name": "cnode1", "node_type": "CNode"}]

        self.publish_activation = False
        self.last_activation = 0.0

        for key, value in params.items():
            setattr(self, key, value)

        # Publish node activation topic (when SetActivationTopic is true)
        self.publish_activation_topic = self.create_publisher(
            Activation,
            'cognitive_node/' + str(name) + '/activation',
            0
        )

        # Get Activation Service
        self.get_activation_service = self.create_service(
            GetActivation,
            'cognitive_node/' + str(name) + '/get_activation',
            self.get_activation_callback
        )
        
        # Get Information Service
        self.get_information_service = self.create_service(
            GetInformation,
            'cognitive_node/' + str(name) + '/get_information',
            self.get_information_callback
        )
        
        # Set Activation Topic Service
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
        keys_to_delete = [key for key in node_data.keys() if key.startswith('_') or 'service' in key or 'topic' in key]
        for key in keys_to_delete:
            del node_data[key]

        optional_keys_to_delete = ['subscription']
        for key in optional_keys_to_delete:
            if key in node_data:
                del node_data[key]

        return node_data
    
    def register_in_LTM(self, data_dic):
        """
        Registers the node in the LTM
        """

        data = yaml.dump({**data_dic, 'activation': self.activation, 'perception': self.perception})

        service_name = 'ltm_0' + '/add_node' # TODO choose LTM ID
        add_node_to_LTM_client = ServiceClient(AddNodeToLTM, service_name)
        ltm_response = add_node_to_LTM_client.send_request(name=self.name, node_type=self.node_type, data=data)
        add_node_to_LTM_client.destroy_node()
        return ltm_response
    
    def remove_from_LTM(self):
        """
        Removes the node from the LTM. Returns true if the operation was succesful, false otherwise.
        """

        service_name = 'ltm_0' + '/delete_node' # TODO: choose the ltm ID
        delete_node_client = ServiceClient(DeleteNodeFromLTM, service_name)
        ltm_response = delete_node_client.send_request(name=self.name)
        delete_node_client.destroy_node()
        return ltm_response.deleted
   
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
            self.publish_activation = True
        else:
            self.publish_activation = False
        response.activation_topic = activation_topic
        return response
    
    def publish(self, message=None, first_time=False):
        """Publish node information."""
        if not message:
            message = self.name + str('_msg')
        if first_time:
            message.command = "new"
        else:
            message.command = "update"
        message.node_name = self.name
        message.neighbor_names = [node.name for node in self.neighbors]
        message.neighbor_types = [node.node_type for node in self.neighbors]
        if isinstance(self.activation, list):
            message.activation = max(self.activation)
        else:
            message.activation = self.activation
        self.node_publisher.publish(message)        
    
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
