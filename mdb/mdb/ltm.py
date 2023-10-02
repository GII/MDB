import yaml

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from mdb.cognitive_node import CognitiveNode

from mdb.send_to_commander_client import SendToCommanderClient

from mdb_interfaces.srv import SendToLTM

class LTM(Node):
    def __init__(self):
        super().__init__('LTM')
        self.cognitive_nodes = {'ANode': {}, 'BNode': {}, 'Policy': {}}
        self.state_publisher = self.create_publisher(String, 'state', 10)
        self.state_timer = self.create_timer(1, self.state_timer_callback)

        self.last_id = 0
        
        # SendToLTM Service for the cognitive nodes and the main loop
        self.send_to_LTM_service = self.create_service(
            SendToLTM,
            'send_to_LTM',
            self.handle_command
        )

    def state_timer_callback(self):
        msg = String()
        msg.data = self.cognitive_nodes.__str__()
        self.state_publisher.publish(msg)
        self.get_logger().info("State: " + str(msg.data) + ".")


    @property
    def a_nodes(self):
        """
        Get all cognitive nodes of type 'ANode' from the LTM.

        :return: A list of 'ANode' nodes.
        :rtype: list
        """
        return self.cognitive_nodes.get('ANode')
    
    @property
    def b_nodes(self):
        """
        Get all cognitive nodes of type 'BNode' from the LTM.

        :return: A list of 'BNode' nodes.
        :rtype: list
        """
        return self.cognitive_nodes.get('BNode')
    
    @property
    def policy_nodes(self):
        """
        Get all cognitive nodes of type 'Policy' from the LTM.

        :return: A list of 'Policy' nodes.
        :rtype: list
        """
        return self.cognitive_nodes.get('Policy')

    def handle_command(self, request, response):
        """
        Handle command requests received from cognitive nodes.

        :param request: The command request.
        :type request: mdb_interfaces.srv.SendToLTM_Request
        :param response: The response to the command.
        :type response: mdb_interfaces.srv.SendToLTM_Response
        :return: The response to the command.
        :rtype: mdb_interfaces.srv.SendToLTM_Response
        """
        command = str(request.command)
        name = str(request.name)
        node_type = str(request.type)

        try:
            if(command == 'register'):

                if self.node_exists(node_type, name):
                    response.msg = str(node_type) + ' ' + str(name) + ' already exists.'
                
                else:
                    data = str(request.data)

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
        :rtype: mdb_interfaces.srv.SendToLTM_Response
        """
        send_to_commander_client = SendToCommanderClient()
        executor_response = send_to_commander_client.send_request(command, self.name, self.type, data)
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
    