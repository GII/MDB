import sys
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from core_interfaces.srv import AddNodeToLTM, DeleteNodeFromLTM, GetNodeFromLTM, ReplaceNodeFromLTM, SetChangesTopic
from cognitive_node_interfaces.srv import AddNeighbor, DeleteNeighbor
from core.service_client import ServiceClient

class LTM(Node):
    """
    The Long-Term Memory (LTM) node in the cognitive architecture.

    This node is responsible for storing and managing cognitive nodes of various types.
    It provides services for adding, replacing, deleting, and retrieving these nodes,
    as well as publishing changes.

    Attributes:
        id (str): An identifier for the LTM instance.
        changes_topic (bool): Flag to indicate if changes are being published.
        cognitive_nodes (dict): A dictionary to store cognitive nodes by type.
        state_publisher (Publisher): Publisher for the state of the LTM.
        add_node_service (Service): Service to add new cognitive nodes.
        replace_node_service (Service): Service to replace existing cognitive nodes.
        delete_node_service (Service): Service to delete cognitive nodes.
        get_node_service (Service): Service to retrieve data of cognitive nodes.
        set_changes_topic_service (Service): Service to set the changes topic.
    """    
    
    def __init__(self, id):
        """
        Initialize the LTM node.

        :param id: The identifier for this LTM instance.
        :type id: str
        """        
        super().__init__('ltm_' + str(id))
        self.id = id
        self.changes_topic = True
        # TODO Remove ANode and BNode
        # TODO Create keys from config file
        self.cognitive_nodes = {'ANode': {}, 'BNode': {}, 'Drive': {}, 'Goal': {}, 'Need': {}, 'Policy': {}, 'Perception': {},'PNode': {}, 'UtilityModel': {}, 'WorldModel': {}}
        
        # State topic
        self.state_publisher = self.create_publisher(
            String, 
            'state',
            10
        )

        # Add node service
        self.add_node_service = self.create_service(
            AddNodeToLTM,
            'ltm_' + str(self.id) + '/add_node',
            self.add_node_callback
        )

        # Replace node service
        self.replace_node_service = self.create_service(
            ReplaceNodeFromLTM,
            'ltm_' + str(self.id) + '/replace_node',
            self.replace_node_callback
        )

        # Delete node service
        self.delete_node_service = self.create_service(
            DeleteNodeFromLTM,
            'ltm_' + str(self.id) + '/delete_node',
            self.delete_node_callback
        )

        # Get node service
        self.get_node_service = self.create_service(
            GetNodeFromLTM,
            'ltm_' + str(self.id) + '/get_node',
            self.get_node_callback
        )

        # Set changes topic service
        self.set_changes_topic_service = self.create_service(
            SetChangesTopic,
            'ltm_' + str(self.id) + '/set_changes_topic',
            self.set_changes_topic_callback
        )

    def publish_state(self):
        if(self.changes_topic):
            msg = String()
            msg.data = self.cognitive_nodes.__str__()
            self.state_publisher.publish(msg)
            self.get_logger().info(f"State: {msg.data}")

    # region Properties
    @property
    def a_nodes(self): # TODO Remove this
        """
        Get all cognitive nodes of type 'ANode' from the LTM.

        :return: A list of 'ANode' nodes.
        :rtype: list
        """
        return self.cognitive_nodes.get('ANode', [])
    
    @property
    def b_nodes(self): # TODO Remove this
        """
        Get all cognitive nodes of type 'BNode' from the LTM.

        :return: A list of 'BNode' nodes.
        :rtype: list
        """
        return self.cognitive_nodes.get('BNode', [])
    
    @property
    def drives(self):
        """
        Get all cognitive nodes of type 'Drive' from the LTM.

        :return: A list of 'Drive' nodes.
        :rtype: list
        """
        return self.cognitive_nodes.get('Drive', [])
    
    @property
    def goals(self):
        """
        Get all cognitive nodes of type 'Goal' from the LTM.

        :return: A list of 'Goal' nodes.
        :rtype: list
        """
        return self.cognitive_nodes.get('Goal', [])
    
    @property
    def needs(self):
        """
        Get all cognitive nodes of type 'Need' from the LTM.

        :return: A list of 'Need' nodes.
        :rtype: list
        """
        return self.cognitive_nodes.get('Need', [])
    
    @property
    def policies(self):
        """
        Get all cognitive nodes of type 'Policy' from the LTM.

        :return: A list of 'Policy' nodes.
        :rtype: list
        """
        return self.cognitive_nodes.get('Policy', [])

    @property
    def pnodes(self):
        """
        Get all cognitive nodes of type 'PNode' from the LTM.

        :return: A list of 'PNode' nodes.
        :rtype: list
        """
        return self.cognitive_nodes.get('PNode', [])

    @property
    def utilitymodels(self):
        """
        Get all cognitive nodes of type 'UtilityModel' from the LTM.

        :return: A list of 'UtilityModel' nodes.
        :rtype: list
        """
        return self.cognitive_nodes.get('UtilityModel', [])
    
    @property
    def worldmodels(self):
        """
        Get all cognitive nodes of type 'WorldModel' from the LTM.

        :return: A list of 'WorldModel' nodes.
        :rtype: list
        """
        return self.cognitive_nodes.get('WorldModel', [])

    # endregion Properties
    
    # region Callbacks
    def add_node_callback(self, request, response): 
        """
        Callback function for the 'add_node' service.
        Adds a cognitive node to the LTM.

        This method checks if the node already exists in the LTM. If it does, it sets the response
        'added' attribute to False. If the node does not exist, it adds the new node to the LTM 
        and sets the 'added' attribute to True.

        :param request: The service request containing the node's name, type, and data.
        :type request: AddNodeToLTM_Request
        :param response: The service response.
        :type response: AddNodeToLTM_Response
        :return: The response indicating whether the node was added successfully.
        :rtype: AddNodeToLTM_Response
        """
        name = str(request.name)
        node_type = str(request.node_type)
        
        if self.node_exists(node_type, name):
            self.get_logger().info(f"{node_type} {name} already exists.")
            response.added = False

        else:
            data = str(request.data)
            data_dic = yaml.load(data, Loader=yaml.FullLoader)
            self.add_node(node_type, name, data_dic)     
            self.get_logger().info(f"Added {node_type} {name}")
            response.added = True

        return response
    
    def replace_node_callback(self, request, response):
        """
        Callback function for the 'replace_node' service.
        Replaces an existing cognitive node in the LTM.

        This method first checks if the original node exists in the LTM. If it doesn't, it sets the 
        response 'replaced' attribute to False. If the original node exists, it then checks if the new 
        name is already taken. If not, it replaces the node with the new data and sets the 'replaced' 
        attribute to True.

        :param request: The service request containing the original and new name of the node, its type, 
                        and the new data for the node.
        :type request: ReplaceNodeFromLTM_Request
        :param response: The service response.
        :type response: ReplaceNodeFromLTM_Response
        :return: The response indicating whether the node was replaced successfully.
        :rtype: ReplaceNodeFromLTM_Response
        """
        name = str(request.name)
        new_name = str(request.new_name)
        node_type = str(request.node_type)
        
        if not self.node_exists(node_type, name):
            self.get_logger().info(f"{node_type} {name} doesn't exist.")
            response.replaced = False

        elif self.node_exists(node_type, new_name):
            self.get_logger().info(f"{node_type} {name} already exists.")
            response.replaced = False

        else:
            data = str(request.data)
            data_dic = yaml.load(data, Loader=yaml.FullLoader)
            self.add_node(node_type, name, data_dic)     
            self.get_logger().info(f"Replaced {node_type} {name} with {node_type} {name}.")
            response.replaced = True

        return response
    
    def delete_node_callback(self, request, response):
        """
        Callback function for the 'delete_node' service.
        Deletes a cognitive node from the LTM.

        This method iterates over all node types in the LTM to find the node with the given name. 
        If the node is found, it is deleted, and the response 'deleted' attribute is set to True. 
        If the node is not found, the 'deleted' attribute is set to False.

        :param request: The service request containing the name of the node to be deleted.
        :type request: DeleteNodeFromLTM_Request
        :param response: The service response.
        :type response: DeleteNodeFromLTM_Response
        :return: The response indicating whether the node was deleted successfully.
        :rtype: DeleteNodeFromLTM_Response
        """
        name = str(request.name)
        for node_type in self.cognitive_nodes:
            if name in self.cognitive_nodes[node_type]:
                self.delete_node(node_type, name)
                self.get_logger().info(f"{node_type} {name} deleted from LTM.")
                response.deleted = True
                return response

        self.get_logger().info(f"Node {name} doesn't exist.")
        response.deleted = False
        return response
    
    def get_node_callback(self, request, response):
        """
        Callback function for the 'get_node' service.
        Retrieves data of a specific cognitive node from the LTM.

        This method iterates over all node types in the LTM to find the node with the given name. 
        If the node is found, its data is serialized into YAML format and returned in the response. 
        If the node is not found, an empty string is returned.

        :param request: The service request containing the name of the node to retrieve.
        :type request: GetNodeFromLTM_Request
        :param response: The service response containing the node data if found.
        :type response: GetNodeFromLTM_Response
        :return: The response with the node data in YAML format or an empty string.
        :rtype: GetNodeFromLTM_Response
        """        
        name = str(request.name)

        if name == "": #Return dict with all nodes if empty string is passed
            data_dic = self.cognitive_nodes
            data= yaml.dump(data_dic)
            self.get_logger().info(f"Sending all nodes in LTM: {self.id}")
            response.data=data
            return response

        else:
            for node_type in self.cognitive_nodes:
                if name in self.cognitive_nodes[node_type]:
                    data_dic = self.cognitive_nodes[node_type][name]
                    data = yaml.dump(data_dic)
                    self.get_logger().info(f"{node_type} {name}: {data}")
                    response.data = data
                    return response
            self.get_logger().info(f"{node_type} {name} doesn't exist.")
            response.data = ""
            return response
    
    def set_changes_topic_callback(self, request, response):
        """
        Callback function for the 'set_changes_topic' service.
        Sets the topic for tracking changes in the LTM.

        This method updates the 'changes_topic' attribute of the LTM with the provided topic name 
        from the service request. The updated topic name is returned in the service response.

        :param request: The service request containing the name of the changes topic to be set.
        :type request: SetChangesTopic_Request
        :param response: The service response confirming the updated changes topic.
        :type response: SetChangesTopic_Response
        :return: The response with the updated changes topic name.
        :rtype: SetChangesTopic_Response
        """        
        changes_topic = request.changes_topic
        self.changes_topic = changes_topic
        self.get_logger().info(f"Changes topic set to {changes_topic}")
        response.changes_topic = changes_topic
        return response
    
    # endregion Callbacks
    
    # region CRUD operations
    def add_node(self, node_type, node_name, node_data):
        """
        Add a cognitive node to the LTM.

        :param node_type: The type of the cognitive node.
        :type node_type: str
        :param node_name: The name of the cognitive node.
        :type node_name: str
        :param node_data: The dictionary containing the data of the cognitive node.
        :type node_data: dict
        """
        self.cognitive_nodes[node_type][node_name] = node_data

        #TODO: Neighbor handling

        if not node_data['neighbors']:
            if node_type=='Perception':
                goals= [{'name': goal, 'node_type': 'Goal'} for goal in self.cognitive_nodes['Goal']]
                world_models= [{'name': WM, 'node_type': 'WorldModel'} for WM in self.cognitive_nodes['WorldModel']]
                policies= [{'name': policy, 'node_type': 'Policy'} for policy in self.cognitive_nodes['Policy']]
                neighbors=goals+world_models+policies
                self.cognitive_nodes[node_type][node_name]['neighbors']=neighbors
            else:
                neighbors=[{'name': perception, 'node_type': 'Perception'} for perception in self.cognitive_nodes['Perception']]
                self.cognitive_nodes[node_type][node_name]['neighbors']=neighbors 


        for neighbor in node_data['neighbors']:
            self.add_neighbor(node_name,node_type,neighbor) 
        self.publish_state()
    
    def delete_node(self, node_type, node_name):
        """
        Delete a cognitive node from the LTM.

        :param node_type: The type of the cognitive node.
        :type node_type: str
        :param node_name: The name of the cognitive node.
        :type node_name: str
        """
        del self.cognitive_nodes[node_type][node_name]
        self.publish_state()


    def node_exists(self, node_type, node_name):
        """
        Check if a cognitive node exists in the LTM.

        :param node_type: The type of the cognitive node.
        :type node_type: str
        :param node_name: The name of the cognitive node.
        :type node_name: str
        :return: True if the node exists, False otherwise.
        :rtype: bool
        """
        if node_type in self.cognitive_nodes:
            return node_name in self.cognitive_nodes[node_type]
        return False
    
    def node_type_exists(self, node_type):
        """
        Check if cognitive nodes of a specific type exist in the LTM.

        :param node_type: The type of cognitive node to check.
        :type node_type: str
        :return: True if the specified type exist, False otherwise.
        :rtype: bool
        """
        return node_type in self.cognitive_nodes
    
    # endregion CRUD operations

    def add_neighbor(self, neighbor_name, neighbor_type, service_node_name): #TODO Implement
        service_name = 'cognitive_node/' + service_node_name + '/add_neighbor'
        neighbor_client=ServiceClient(AddNeighbor, service_name)
        result=neighbor_client.send_request(neighbor_name=neighbor_name, neighbor_type=neighbor_type)
        return result

def main(args=None):
    rclpy.init()
    id = int(sys.argv[1])
    ltm = LTM(id)

    rclpy.spin(ltm)
    rclpy.shutdown()

    ltm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    