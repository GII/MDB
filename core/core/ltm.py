import yaml

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from core.cognitive_node import CognitiveNode

from core_interfaces.srv import SendToLTM, SendToCommander
from core_interfaces.srv import AddNodeToLTM, DeleteNodeFromLTM, GetNodeFromLTM, ReplaceNodeFromLTM

class LTM(Node):
    def __init__(self):
        super().__init__('LTM')
        self.id = 0
        # TODO Remove ANode and BNode
        self.cognitive_nodes = {'ANode': {}, 'BNode': {}, 'Drive': {}, 'Goal': {}, 'Need': {}, 'Policy': {}, 'Perception': {},'PNode': {}, 'UtilityModel': {}, 'WorldModel': {}}
        self.state_publisher = self.create_publisher(String, 'state', 10)
        self.state_timer = self.create_timer(1, self.state_timer_callback)

        self.last_id = 0
        
        # SendToLTM Service for the cognitive nodes and the main loop
        self.send_to_LTM_service = self.create_service(
            SendToLTM,
            'send_to_LTM',
            self.handle_command
        )

        # N: Add node service
        self.add_node_service = self.create_service(
            AddNodeToLTM,
            'ltm_' + str(self.id) + '/add_node',
            self.add_node_callback
        )

        # N: Replace node service
        self.replace_node_service = self.create_service(
            ReplaceNodeFromLTM,
            'ltm_' + str(self.id) + '/replace_node',
            self.replace_node_callback
        )

        # N: Delete node service
        self.delete_node_service = self.create_service(
            DeleteNodeFromLTM,
            'ltm_' + str(self.id) + '/delete_node',
            self.delete_node_callback
        )

        # N: Get node service
        self.get_node_service = self.create_service(
            GetNodeFromLTM,
            'ltm_' + str(self.id) + '/get_node',
            self.get_node_callback
        )

    def state_timer_callback(self):
        msg = String()
        msg.data = self.cognitive_nodes.__str__()
        self.state_publisher.publish(msg)
        self.get_logger().info("State: " + str(msg.data) + ".")


    @property
    def a_nodes(self): # TODO Remove this
        """
        Get all cognitive nodes of type 'ANode' from the LTM.

        :return: A list of 'ANode' nodes.
        :rtype: list
        """
        return self.cognitive_nodes.get('ANode')
    
    @property
    def b_nodes(self): # TODO Remove this
        """
        Get all cognitive nodes of type 'BNode' from the LTM.

        :return: A list of 'BNode' nodes.
        :rtype: list
        """
        return self.cognitive_nodes.get('BNode')
    @property
    def drives(self):
        """
        Get all cognitive nodes of type 'Drive' from the LTM.

        :return: A list of 'Drive' nodes.
        :rtype: list
        """
        return self.cognitive_nodes.get('Drive')
    
    @property
    def goals(self):
        """
        Get all cognitive nodes of type 'Goal' from the LTM.

        :return: A list of 'Goal' nodes.
        :rtype: list
        """
        return self.cognitive_nodes.get('Goal')
    
    @property
    def needs(self):
        """
        Get all cognitive nodes of type 'Need' from the LTM.

        :return: A list of 'Need' nodes.
        :rtype: list
        """
        return self.cognitive_nodes.get('Need')
    
    @property
    def policies(self):
        """
        Get all cognitive nodes of type 'Policy' from the LTM.

        :return: A list of 'Policy' nodes.
        :rtype: list
        """
        return self.cognitive_nodes.get('Policy')

    @property
    def pnodes(self):
        """
        Get all cognitive nodes of type 'PNode' from the LTM.

        :return: A list of 'PNode' nodes.
        :rtype: list
        """
        return self.cognitive_nodes.get('PNode')

    @property
    def utilitymodels(self):
        """
        Get all cognitive nodes of type 'UtilityModel' from the LTM.

        :return: A list of 'UtilityModel' nodes.
        :rtype: list
        """
        return self.cognitive_nodes.get('UtilityModel')
    
    @property
    def worldmodels(self):
        """
        Get all cognitive nodes of type 'WorldModel' from the LTM.

        :return: A list of 'WorldModel' nodes.
        :rtype: list
        """
        return self.cognitive_nodes.get('WorldModel')

    def add_node_callback(self, request, response): 
        name = str(request.name)
        self.get_logger().info('Adding node ' + name + '...')
        # TODO: implement logic
        response.added = True
        return response
    
    def replace_node_callback(self, request, response): # TODO: implement
        name = str(request.name)
        self.get_logger().info('Replacing node ' + name + '...')
        # TODO: implement logic
        response.replaced = True
        return response
    
    def delete_node_callback(self, request, response): # TODO: implement
        name = str(request.name)
        self.get_logger().info('Deleting node ' + name + '...')
        # TODO: implement logic
        response.deleted = True
        return response
    
    def get_node_callback(self, request, response): # TODO: implement
        name = str(request.name)
        self.get_logger().info('Getting node ' + name + '...')
        # TODO: implement logic
        response.data = 'Node data'
        return response
    
    def set_changes_topic_callback(self, request, response): # TODO: implement
        changes_topic = request.changes_topic
        self.get_logger().info('Setting changes topic to ' + str(changes_topic) + '...')
        # TODO: implement logic
        response.changes_topic = changes_topic
        return response

    def handle_command(self, request, response):
        """
        Handle command requests received from cognitive nodes.

        :param request: The command request.
        :type request: core_interfaces.srv.SendToLTM_Request
        :param response: The response to the command.
        :type response: core_interfaces.srv.SendToLTM_Response
        :return: The response to the command.
        :rtype: core_interfaces.srv.SendToLTM_Response
        """

        command = str(request.command)
        name = str(request.name)
        node_type = str(request.node_type)

        self.get_logger().info('Handling command: ' + str(command))
        self.get_logger().info('name: ' + str(name))
        self.get_logger().info('node_type: ' + str(node_type))
    
        try:
            if(command == 'register'):

                if self.node_exists(node_type, name):
                    response.msg = str(node_type) + ' ' + str(name) + ' already exists.'
                
                else:
                    data = str(request.data)
                    self.get_logger().info('Data: ' + data + '.')

                    data_dic = yaml.load(data, Loader=yaml.FullLoader)

                    publishing_topics = data_dic['publishing']
                    subscribed_topics = data_dic['subscribed']

                    self.add_node(node_type, name, publishing_topics, subscribed_topics)     
                                        
                    self.get_logger().info('Registered node ' + str(name) + '.')

                    response.msg = 'Registered node ' + str(name) + '.'
            
            elif(command == 'delete'):

                if not self.node_exists(node_type, name):
                    response.msg = str(node_type) + ' ' + str(name) + ' does not exist.'
                
                else:
                                      
                    self.delete_node(node_type, name)
                    self.get_logger().info(str(node_type) + ' ' + str(name) + ' deleted.')

                    response.msg = str(name) + ' deleted.'
            
            elif(command == 'subscribe'):

                if not self.node_exists(node_type, name):
                    response.msg = str(node_type) + ' ' + str(name) + ' does not exist.'
                
                else:

                    # TODO: send request to commander
                    self.get_logger().info(str(node_type) + ' ' + str(name) + ' deleted.')

                    response.msg = str(name) + ' deleted.'
            
            elif (command == 'publish'):

                if not self.node_exists(node_type, name):
                    response.msg = str(node_type) + ' ' + str(name) + ' does not exist.'
                
                else:
                                      
                    topic = str(request.data)
                    commander_response = self.send_request_to_commander(command, name, node_type, topic)
                                        
                    self.get_logger().info(str(node_type) + ' ' + str(name) + ' now publishing in topic ' + str(topic) + '.')

                    response.msg = str(name) + ' now publishing in topic ' + str(topic) + '.'
                    
            else:
                self.get_logger().info('Wrong command.')
                response.msg = 'Wrong request: ' + str(command) + '.'
        
        except Exception as e:
            error = 'Error handling command: ' + str(e)
            self.get_logger().error(error)
            response.msg = error
        
        return response
    
    def send_request_to_commander(self, command, data):
        """
        Send a request to the commander node.

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
        service_name = 'send_to_commander'
        send_to_commander_client = ServiceClient(SendToCommander, service_name)
        executor_response = send_to_commander_client.send_request(command=command, name=self.name, node_type=self.type, data=data)
        send_to_commander_client.destroy_node()
        return executor_response
    
    def add_node(self, node_type, node_name, publishing_topics=[], subscribed_topics=[]):
        """
        Add a cognitive node to the LTM.

        :param node_type: The type of the cognitive node.
        :type node_type: str
        :param node_name: The name of the cognitive node.
        :type node_name: str
        :param publishing_topics: The list of topics where the node is publishing.
        :type publishing_topics: list[str]
        :param subscribed_topics: The list of topics to which the node is subscribed.
        :type subscribed_topics: list[str]
        """
        node_info = {
            'subscribed': subscribed_topics,
            'publishing': publishing_topics,
        }
        self.cognitive_nodes[node_type][node_name] = node_info
    
    def delete_node(self, node_type, node_name):
        """
        Remove a node from the LTM.

        :param node_type: The type of the node to remove.
        :type node_type: str
        :param node_name: The name of the node to remove.
        :type node_name: str
        :return: True if the node was successfully removed, False otherwise.
        :rtype: bool
        """
        if node_type in self.cognitive_nodes and node_name in self.cognitive_nodes[node_type]:
            del self.cognitive_nodes[node_type][node_name]
            return True
        else:
            return False        

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

    def get_subscriptions(self, node_type, node_name):
        """
        Get the list of topics to which a cognitive node is subscribed.

        :param node_type: The type of the cognitive node.
        :type node_type: str
        :param node_name: The name of the cognitive node.
        :type node_name: str
        :return: A list of topics to which the node is subscribed, or an empty list if the node doesn't exist.
        :rtype: list[str]
        """
        if self.node_exists(node_type, node_name):
            return self.cognitive_nodes[node_type][node_name]['subscribed']
        return []
    
    def get_publications(self, node_type, node_name):
        """
        Get the list of topics where a cognitive node is publishing.

        :param node_type: The type of the cognitive node.
        :type node_type: str
        :param node_name: The name of the cognitive node.
        :type node_name: str
        :return: A list of topics where the node is publishing, or an empty list if the node doesn't exist.
        :rtype: list[str]
        """
        if self.node_exists(node_type, node_name):
            return self.cognitive_nodes[node_type][node_name]['publishing']
        return []
    
    def add_subscription(self, node_type, node_name, topic):
        """
        Add a topic subscription to a cognitive node.

        :param node_type: The type of the cognitive node.
        :type node_type: str
        :param node_name: The name of the cognitive node.
        :type node_name: str
        :param topic: The topic to subscribe to.
        :type topic: str
        """
        if self.node_exists(node_type, node_name):
            self.cognitive_nodes[node_type][node_name]['subscribed'].append(topic)

    def add_publication(self, node_type, node_name, topic):
        """
        Add a topic publication to a cognitive node.

        :param node_type: The type of the cognitive node.
        :type node_type: str
        :param node_name: The name of the cognitive node.
        :type node_name: str
        :param topic: The topic to publish to.
        :type topic: str
        """
        if self.node_exists(node_type, node_name):
            self.cognitive_nodes[node_type][node_name]['publishing'].append(topic)
        else:
            self.cognitive_nodes.setdefault(node_type, {}).setdefault(node_name, {'subscribed': [], 'publishing': [topic]})


def main(args=None):
    rclpy.init()
    ltm = LTM()

    rclpy.spin(ltm)
    rclpy.shutdown()

    ltm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    