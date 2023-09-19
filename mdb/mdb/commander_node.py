import sys
import rclpy
import random
from rclpy.node import Node

from mdb.send_to_executor_client import SendToExecutorClient
from mdb_interfaces.srv import SendToCommander

class CommanderNode(Node):
    """
    This class is responsible for handling the commands from the cognitive nodes and sending them to the execution nodes.

    Attributes:
        cli (:class:`rclpy.client.Client`): A client for the 'SendCommand' service.

    """

    def __init__(self):
        """
        Constructor for the CommanderNode class.

        Creates a ROS 2 node named 'commander node' and a service for the cognitive nodes.
        """
        super().__init__('commander_node')
        self.executor_ids = [0, 1] # TODO configure this from file
        self.nodes = {}

        for executor_id in self.executor_ids:
            self.nodes[executor_id] = []
                
        # Create Command Service for the cognitive nodes
        self.send_to_commander_service = self.create_service(
            SendToCommander,
            'send_to_commander_service',
            self.handle_command
        )

    def handle_command(self, request, response):
        """
        Handle command requests received from cognitive nodes and send them to the correct executor node.

        :param request: The command request.
        :type request: mdb_interfaces.srv.SendToCommander_Request
        :param response: The response to the command.
        :type response: mdb_interfaces.srv.SendToCommander_Response
        :return: The response to the command.
        :rtype: mdb_interfaces.srv.SendToCommander_Response
        """
        command = str(request.command)
        name = str(request.name)
        try:
            if(command == 'create'):

                if self.node_exists(name):
                    response.msg = 'Node with name ' + str(name) + ' already exists.'
                
                else:
                    type = str(request.type)
                    data = str(request.data)
                    
                    ex = self.get_lowest_load_executor()
                    
                    executor_response = self.send_request_to_executor(ex, command, name, type, data)
                    
                    self.register_node(ex, name)
                    
                    self.get_logger().info('Node ' + str(name) + ' created in executor ' + str(ex) + '.')

                    response.msg = 'Node ' + str(name) + ' created.'
            
            elif (command == 'read'):

                if not self.node_exists(name):
                    response.msg = 'Node with name ' + str(name) + ' does not exist.'
                
                else:
                    ex = self.get_executor_for_node(name)

                    executor_response = self.send_request_to_executor(ex, command, name, '', '')

                    self.get_logger().info('Read node ' + str(name) + ': ' + str(executor_response.msg) + '.')

                    response.msg = executor_response.msg

            elif (command == 'delete'):

                if not self.node_exists(name):
                    response.msg = 'Node with name ' + str(name) + ' does not exist.'
                
                else:
                    ex = self.get_executor_for_node(name)

                    executor_response = self.send_request_to_executor(ex, command, name, '', '')

                    self.get_logger().info('Node ' + str(name) + ' deleted from executor ' + str(ex) + '.')

                    self.remove_node_from_executor(ex, name)

                    response.msg = 'Deleted node ' + str(name) + '.'

            elif (command == 'save'):
                
                if not self.node_exists(name):
                    response.msg = 'Node with name ' + str(name) + ' does not exist.'
                
                else:
                    ex = self.get_executor_for_node(name)

                    executor_response = self.send_request_to_executor(ex, command, name, '', '')

                    response.msg = 'Node saved: ' + str(name) + '.'

            elif (command == 'load'):

                if self.node_exists(name):
                    response.msg = 'Node with name ' + str(name) + ' already exists.'
                
                else:
                    type = str(request.type) # TODO save the type of the node
                    
                    ex = self.get_lowest_load_executor()
                    
                    executor_response = self.send_request_to_executor(ex, command, name, type, '')
                    
                    self.register_node(ex, name)
                    
                    self.get_logger().info('Node ' + str(name) + ' loaded in executor ' + str(ex) + '.')

                    response.msg = 'Node ' + str(name) + ' loaded.'
                    
            else:
                self.get_logger().info('Wrong command.')
                response.msg = 'Wrong request: ' + str(command) + '.'
        
        except Exception as e:
            error = 'Error handling command: ' + str(e)
            self.get_logger().error(error)
            response.msg = error
        
        return response
    
    # TODO: implement this method with load balancing
    def get_lowest_load_executor(self):
        """
        Get the executor with the lowest load.

        :return: The executor with the lowest load.
        :rtype: int
        """
        ex = random.randint(0, self.executor_ids.__len__() -1)
        self.get_logger().info('Lowest load executor: ' + str(ex))
        return ex
    
    def get_executor_for_node(self, name):
        """
        Get the executor that manages a specific node.

        :param name: The name of the node.
        :type name: str
        :return: The executor ID or None if the node is not found.
        :rtype: int or None
        """
        for executor_id, cognitive_nodes in self.nodes.items():
            if name in cognitive_nodes:
                return executor_id
        return None
    
    def node_exists(self, name):
        """
        Check if a cognitive node with the given name exists.

        :param name: The name of the node.
        :type name: str
        :return: True if the node exists, False otherwise.
        :rtype: bool
        """
        for cognitive_nodes in self.nodes.values():
            if name in cognitive_nodes:
                return True
        return False    
    
    def register_node(self, executor_id, node_name):
        """
        Register a node in the list of managed nodes for an executor.

        :param executor_id: The ID of the executor.
        :type executor_id: int
        :param node_name: The name of the node.
        :type node_name: str
        """
        self.nodes[executor_id].append(node_name)

    def remove_node_from_executor(self, executor_id, node_name):
        """
        Remove a node from the list of managed nodes for an executor.

        :param executor_id: The ID of the executor.
        :type executor_id: int
        :param node_name: The name of the node to remove.
        :type node_name: str
        """
        self.nodes[executor_id].remove(node_name)

    def send_request_to_executor(self, executor_id, command, name, type, data):
        """
        Send a request to an executor for node management.

        :param executor_id: The ID of the executor.
        :type executor_id: int
        :param command: The command to send.
        :type command: str
        :param name: The name of the node.
        :type name: str
        :param type: The type of the node.
        :type type: str
        :param data: Optional data to initialize the node.
        :type data: str
        :return: The response from the executor.
        :rtype: mdb_interfaces.srv.SendToExecutor_Response
        """
        create_command_client = SendToExecutorClient(executor_id)
        executor_response = create_command_client.send_request(command, name, type, data)
        create_command_client.destroy_node()
        return executor_response


def main(args=None):
    rclpy.init()
    commander = CommanderNode()

    rclpy.spin(commander)
    rclpy.shutdown()

    commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()