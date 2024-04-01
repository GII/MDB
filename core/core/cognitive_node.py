import yaml
from rclpy.node import Node

from core.service_client import ServiceClient
from core_interfaces.srv import AddNodeToLTM, DeleteNodeFromLTM
from cognitive_node_interfaces.srv import GetActivation, GetInformation, SetActivationTopic, AddNeighbor, DeleteNeighbor
from cognitive_node_interfaces.msg import Activation
from cognitive_node_interfaces.msg import Perception
from cognitive_node_interfaces.msg import PerceptionParameters

class CognitiveNode(Node):
    """
    A base class for cognitive nodes in the system.

    This class extends the `rclpy.node.Node` class and provides some
    common functionality for cognitive nodes.
    """

    def __init__(self, name, class_name, **params):
        """
        Initialize a CognitiveNode.

        :param name: The name of the node.
        :param class_name: The name of the class, i.e: cognitive_nodes.perception.Perception.
        :param params: Any other attribute of the node.
        """
        super().__init__(name)
        self.name = name
        self.class_name = class_name
        _, _, node_type = self.class_name.rpartition(".")
        self.node_type = node_type

        self.perception = None

        # self.threshold = threshold
        self.neighbors = [] # List of dics, like [{"name": "pnode1", "node_type": "PNode"}, {"name": "cnode1", "node_type": "CNode"}]

        self.activation_topic = False
        self.activation = 0.0

        self.perception = []
        self.threshold = 0.0

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

        #Add Neighbor Service
        self.add_neighbor_service = self.create_service(
            AddNeighbor,
            'cognitive_node/' + str(name) + '/add_neighbor',
            self.add_neighbor_callback
        )

        #Delete Neighbor Service
        self.delete_neighbor_service = self.create_service(
            DeleteNeighbor,
            'cognitive_node/' + str(name) + '/delete_neighbor',
            self.delete_neighbor_callback
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
        
        # delete any key starting with '_', any topic and any service.
        keys_to_delete = [key for key in node_data.keys() if key.startswith('_') or 'service' in key or 'topic' in key]
        for key in keys_to_delete:
            del node_data[key]

        # list of other keys to delete (filled manually)
        optional_keys_to_delete = ['subscription']
        for key in optional_keys_to_delete:
            if key in node_data:
                del node_data[key]

        return node_data
    
    def register_in_LTM(self, data_dic):
        """
        Registers the node in the LTM.

        :param data_dic: A dictionary with the data to be saved.
        :type data_dic: dict
        :return: The response from the LTM service.
        :rtype: core_interfaces.srv.AddNodeToLTM_Response
        """

        data = yaml.dump({**data_dic, 'activation': self.activation, 'perception': self.perception, 'neighbors': self.neighbors})

        service_name = 'ltm_0' + '/add_node' # TODO choose LTM ID
        add_node_to_LTM_client = ServiceClient(AddNodeToLTM, service_name)
        ltm_response = add_node_to_LTM_client.send_request(name=self.name, node_type=self.node_type, data=data)
        add_node_to_LTM_client.destroy_node()
        return ltm_response
    
    def remove_from_LTM(self):
        """
        Removes the node from the LTM. 
        :return: True if the operation was succesful, False otherwise.
        :rtype: core_interfaces.srv.DeleteNodeFromLTM_Response
        """

        service_name = 'ltm_0' + '/delete_node' # TODO: choose the ltm ID
        delete_node_client = ServiceClient(DeleteNodeFromLTM, service_name)
        ltm_response = delete_node_client.send_request(name=self.name)
        delete_node_client.destroy_node()
        return ltm_response.deleted
    
    @staticmethod
    def perception_dict_to_msg(perception_dict):
        """
        Transform a perception dictionary into a ROS message

        :param perception_dict: Dictionary that contais the perceptions
        :type perception_dict: dict
        :return: The ROS message with the perception
        :rtype: cognitve_node_interfaces.msg.Perception
        """
        msg = Perception()
        if perception_dict:
            msg.layout.data_offset = 0
            msg.layout.dim = []
            len_float = 8 #bytes
            for sensor, data in perception_dict.items():
                for values in enumerate(data):
                    dimension = PerceptionParameters()
                    dimension.size_stride_units = 'bytes'
                    dimension.sensor = sensor + str(values[0])
                    dimension.labels = list(values[1].keys())
                    dimension.size = len(values[1])*len_float #bytes
                    dimension.stride = len_float #bytes
                    msg.layout.dim.append(dimension)
                    for value in values[1].values():
                        msg.data.append(value)
        else:
            msg.data = []
        return msg
    
    @staticmethod
    def perception_msg_to_dict(msg):
        """
        Transform a ROS message that contains a perception into a dictionary 

        :param msg: The ROS message with the perception
        :type msg: cognitve_node_interfaces.msg.Perception
        :return: The dictionary with the perceptions
        :rtype: dict
        """
        perception_dict = {}
        first_value = 0 
        for dim in msg.layout.dim:
            sensor = dim.sensor[:-1]
            labels = dim.labels
            size = dim.size
            stride = dim.stride
            num_elements = size//stride
            final_value = num_elements + first_value
            values = msg.data[first_value:final_value]
            values_dict = {labels[i]: values[i] for i in range(len(labels))}

            if not sensor in perception_dict.keys():
                perception_dict[sensor] = [values_dict]
            else:
                perception_dict[sensor].append(values_dict)
                
            first_value += num_elements

        return perception_dict
   
    def calculate_activation(self, perception):
        """
        Calculate the node's activation for the given perception.
        :param perception: The perception for which the activation will be calculated.
        :type perception: float
        """
        raise NotImplementedError
    
    def publish_activation(self, activation):
        """
        Publish the activation of this node.
        :param activation: The activation to be published.
        :type activation: float
        """
        msg = Activation()
        msg.activation = activation
        msg.node_name = self.name
        msg.node_type = self.node_type
        self.publish_activation_topic.publish(msg)
        self.get_logger().info("Activation for " + str(msg.node_type) + str(msg.node_name) +
                               ": " + str(msg.activation))
        
    @staticmethod
    def separate_perceptions(perception):
        """
        Separate a dicionary with several percepcions in several ones with one perception

        :param perception: The dictionary with all perceptions
        :type perception: dict
        :return: A list with the several dictionaries
        :rtype: list
        """
        perceptions = []
        for i in range(max([len(sensor) for sensor in perception.values()])):
                perception_line = {} 
                for sensor, value in perception.items():
                    sid = i % len(value)
                    perception_line[sensor + str(sid)] = value[sid]
                perceptions.append(perception_line)

        return perceptions

    def add_neighbor_callback(self, request, response):
        """
        Add a neighbor to the nodes neighbors collection

        :param request: The request that contains the neighbor info
        :type request: cognitive_node_interfaces.srv.AddNeighbor_Request
        :param response: The response that indicates if the neighbor was added
        :type response: cognitive_node_interfaces.srv.AddNeighbor_Response
        :return: The response that indicates if the neighbor was added
        :rtype: cognitive_node_interfaces.srv.AddNeighbor_Response
        """
        name = request.neighbor_name
        type = request.neighbor_type
        neighbor = {'name':name, 'node_type':type}
        self.neighbors.append(neighbor)
        response.added = True
        return response
    
    def delete_neighbor_callback(self, request, response):
        """
        Delete a neighbor to the nodes neighbors collection

        :param request: The request that contains the neighbor info
        :type request: cognitive_node_interfaces.srv.DeleteNeighbor_Request
        :param response: The response that indicates if the neighbor was deleted
        :type response: cognitive_node_interfaces.srv.DeleteNeighbor_Response
        :return: The response that indicates if the neighbor was deleted
        :rtype: cognitive_node_interfaces.srv.DeleteNeighbor_Response
        """
        name = request.neighbor_name
        type = request.neighbor_type
        neighbor_to_delete = {'name':name, 'node_type':type}

        for neighbor in self.neighbors:
            if neighbor == neighbor_to_delete:
                self.neighbors.remove(neighbor)
                response.deleted = True
            else:
                response.deleted = False
        
        return response

        

    def get_activation_callback(self, request, response): 
        """
        Callback method to calculate and return the node's activations.
        This method calculates the activation of the node based on its perception.

        :return: The response with the calculated activation.
        :rtype: cognitive_node_interfaces.srv.GetActivation_Response
        """
        self.get_logger().info('Getting node activation...')
        perception = self.perception_msg_to_dict(request.perception)
        if perception:
            self.perception = perception
        self.calculate_activation(perception)
        response.activation = self.activation
        return response

    def get_information_callback(self, request, response):
        """
        Callback method to get information about the node.

        This method retrieves information about the node, such as its current activation.
        The activation value is included in the response for external queries.

        :return: The response with the node's information.
        :rtype: cognitive_node_interfaces.srv.GetInformation_Response
        """
        self.get_logger().info('Getting node information...')
        response.node_name = self.name
        response.node_type = self.node_type
        response.current_activation = self.activation
        response.neighbors_name = [neighbor["name"] for neighbor in self.neighbors]
        response.neighbors_type = [neighbor["node_type"] for neighbor in self.neighbors]
        self.get_logger().info("The type of the node " + str(response.node_name) + "is " + str(response.node_type) +
                              ". Its last activation is: " + str(response.current_activation) +
                               ". It's neighbors are: " + str(response.neighbors_name) + ". The node" +
                               "type of each neighbor is: " + str(response.neighbors_type))
        return response

    def set_activation_topic_callback(self, request, response):
        """
        Callback method to control activation topic publishing for the node.

        This method toggles the activation topic publishing for the node based on the provided request.

        :param request: True to publish the activation; False otherwise.
        :type request: cognitive_node_interfaces.srv.SetActivationTopic_Request
        :return: True if the node will publish the activation; False otherwise.
        :rtype: cognitive_node_interfaces.srv.SetActivationTopic_Response
        """
        activation_topic = request.activation_topic
        self.get_logger().info('Setting activation topic to ' + str(activation_topic) + '...')
        self.activation_topic = activation_topic
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
