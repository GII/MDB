import sys
import rclpy
import random
from rclpy.node import Node

from mdb.send_to_ltm_client import SendToLTMClient
from mdb.create_node_client import CreateNodeClient
from mdb.read_node_client import ReadNodeClient
from mdb.delete_node_client import DeleteNodeClient
from mdb.load_node_client import LoadNodeClient
from mdb.save_node_client import SaveNodeClient
from mdb_interfaces.srv import CreateNode, ReadNode, DeleteNode, SaveNode, LoadNode

class CommanderNode(Node):
    """
    This class is responsible for handling the requests from the user and sending them to the execution nodes.

    Attributes:
        cli (:class:`rclpy.client.Client`): A client for the 'SendCommand' service.

    """

    def __init__(self):
        """
        Constructor for the CommanderNode class.

        Creates a ROS 2 node named 'commander node' and a service for the user to send commands.
        """
        super().__init__('commander_node')
        self.executor_ids = [0, 1] # TODO configure this from file
        self.nodes = {}

        for executor_id in self.executor_ids:
            self.nodes[executor_id] = []
                
        # Create Node Service for the user
        self.create_node_service = self.create_service(
            CreateNode,
            'commander/create',
            self.create_node
        )
                
        # Read Node Service for the user
        self.read_node_service = self.create_service(
            ReadNode,
            'commander/read',
            self.read_node
        )
                
        # Delete Node Service for the user
        self.delete_node_service = self.create_service(
            DeleteNode,
            'commander/delete',
            self.delete_node
        )
                
        # Save Node Service for the user
        self.save_node_service = self.create_service(
            SaveNode,
            'commander/save',
            self.save_node
        )

        # Load Node Service for the user
        self.load_node_service = self.create_service(
            LoadNode,
            'commander/load',
            self.load_node
        )

    def create_node(self, request, response):
        """
        Handle the creation of a cognitive node.

        :param request: The creation request.
        :type request: mdb_interfaces.srv.CreateNode_Request
        :param response: The response to the creation request.
        :type response: mdb_interfaces.srv.CreateNode_Response
        :return: The response to the creation request.
        :rtype: mdb_interfaces.srv.CreateNode_Response
        """
        name = str(request.name)
        node_type = str(request.node_type)
        self.get_logger().info('Creating new ' + node_type + ' ' + name + '...')
        if self.node_exists(name):
            self.get_logger().info('Node ' + str(name) + ' already exists.')
            response.created = False
        else:
           
            ex = self.get_lowest_load_executor()
            
            executor_response = self.send_create_request_to_executor(ex, name, node_type)
            
            self.register_node(ex, name)
            
            self.get_logger().info('Node ' + str(name) + ' created in executor ' + str(ex) + '.')

            response = executor_response
        return response

    def read_node(self, request, response):
        """
        Handle reading data from a cognitive node.

        :param request: The read request.
        :type request: mdb_interfaces.srv.ReadNode_Request
        :param response: The response to the read request.
        :type response: mdb_interfaces.srv.ReadNode_Response
        :return: The response to the read request.
        :rtype: mdb_interfaces.srv.ReadNode_Response
        """        
        name = str(request.name)
        
        self.get_logger().info('Reading node ' + name + '...')

        if not self.node_exists(name):
            response.data = ''
            self.get_logger().info('Node ' + name + ' does not exist.')
        
        else:
            ex = self.get_executor_for_node(name)

            executor_response = self.send_read_request_to_executor(ex, name)

            self.get_logger().info('Read node ' + str(name) + ': ' + str(executor_response.data) + '.')

            response = executor_response
        return response

    def delete_node(self, request, response):
        """
        Handle the deletion of a cognitive node.

        :param request: The delete request.
        :type request: mdb_interfaces.srv.DeleteNode_Request
        :param response: The response to the delete request.
        :type response: mdb_interfaces.srv.DeleteNode_Response
        :return: The response to the delete request.
        :rtype: mdb_interfaces.srv.DeleteNode_Response
        """        
        name = str(request.name)
        node_type = str(request.node_type)

        self.get_logger().info('Deleting node ' + name + '...')
        
        if not self.node_exists(name):
            self.get_logger().info('Node ' + name + ' does not exist.')
            response.deleted = False
        
        else:
            node_type = str(request.node_type)

            ex = self.get_executor_for_node(name)

            executor_response = self.send_delete_request_to_executor(ex, name, node_type)

            ltm_response = self.send_request_to_ltm('delete', name, node_type, '') # TODO obtain node_type

            self.get_logger().info('Node ' + str(name) + ' deleted from executor ' + str(ex) + '.')

            self.remove_node_from_executor(ex, name)

            response = executor_response

        return response

    def save_node(self, request, response):
        """
        Handle saving the state of a cognitive node.

        :param request: The save request.
        :type request: mdb_interfaces.srv.SaveNode_Request
        :param response: The response to the save request.
        :type response: mdb_interfaces.srv.SaveNode_Response
        :return: The response to the save request.
        :rtype: mdb_interfaces.srv.SaveNode_Response
        """        
        name = str(request.name)
                
        if not self.node_exists(name):
            self.get_logger().info('Node ' + str(name) + ' does not exist.')
            response.saved = False
        
        else:
            ex = self.get_executor_for_node(name)

            executor_response = self.send_save_request_to_executor(ex, name)
            if executor_response.saved:
                self.get_logger().info('Node ' + str(name) + ' saved.')

            response = executor_response

        return response

    def load_node(self, request, response):
        """
        Handle loading a cognitive node from a configuration.

        :param request: The load request.
        :type request: mdb_interfaces.srv.LoadNode_Request
        :param response: The response to the load request.
        :type response: mdb_interfaces.srv.LoadNode_Response
        :return: The response to the load request.
        :rtype: mdb_interfaces.srv.LoadNode_Response
        """
        name = str(request.name)

        if self.node_exists(name):
            response.loaded = False
        
        else:
            
            ex = self.get_lowest_load_executor()
            
            executor_response = self.send_load_request_to_executor(ex, name)
            
            self.register_node(ex, name)
            
            if executor_response.loaded:
                self.get_logger().info('Node ' + str(name) + ' loaded in executor ' + str(ex) + '.')

            response = executor_response
                    
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

    def send_request_to_ltm(self, command, name, type, data):
        send_to_ltm_client = SendToLTMClient()
        ltm_response = send_to_ltm_client.send_request(command, name, type, data)
        send_to_ltm_client.destroy_node()
        return ltm_response

    def send_create_request_to_executor(self, executor_id, name, node_type):
        """
        Send a 'create' request to an executor.

        :param executor_id: The ID of the executor.
        :type executor_id: int
        :param name: The name of the node to create.
        :type name: str
        :param node_type: The type of the node to create.
        :type node_type: str
        :return: The response from the executor.
        :rtype: mdb_interfaces.srv.CreateNode_Response
        """
        service_name = 'executor' + str(executor_id) + '/create'
        create_client = CreateNodeClient(service_name)
        executor_response = create_client.send_request(name, node_type)
        create_client.destroy_node()
        return executor_response

    def send_read_request_to_executor(self, executor_id, name):
        """
        Send a 'read' request to an executor.

        :param executor_id: The ID of the executor.
        :type executor_id: int
        :param name: The name of the node to read.
        :type name: str
        :return: The response from the executor.
        :rtype: mdb_interfaces.srv.ReadNode_Response
        """
        service_name = 'executor' + str(executor_id) + '/read'
        read_client = ReadNodeClient(service_name)
        executor_response = read_client.send_request(name)
        read_client.destroy_node()
        return executor_response

    def send_delete_request_to_executor(self, executor_id, name, node_type):
        """
        Send a 'delete' request to an executor.

        :param executor_id: The ID of the executor.
        :type executor_id: int
        :param name: The name of the node to delete.
        :type name: str
        :param node_type: The type of the node to delete.
        :type node_type: str
        :return: The response from the executor.
        :rtype: mdb_interfaces.srv.DeleteNode_Response
        """
        service_name = 'executor' + str(executor_id) + '/delete'
        delete_client = DeleteNodeClient(service_name)
        executor_response = delete_client.send_request(name, node_type)
        delete_client.destroy_node()
        return executor_response

    def send_save_request_to_executor(self, executor_id, name):
        """
        Send a 'save' request to an executor.

        :param executor_id: The ID of the executor.
        :type executor_id: int
        :param name: The name of the node to save.
        :type name: str
        :return: The response from the executor.
        :rtype: mdb_interfaces.srv.SaveNode_Response
        """
        service_name = 'executor' + str(executor_id) + '/save'
        save_client = SaveNodeClient(service_name)
        executor_response = save_client.send_request(name)
        save_client.destroy_node()
        return executor_response

    def send_load_request_to_executor(self, executor_id, name):
        """
        Send a 'load' request to an executor.

        :param executor_id: The ID of the executor.
        :type executor_id: int
        :param name: The name of the node to load.
        :type name: str
        :return: The response from the executor.
        :rtype: mdb_interfaces.srv.LoadNode_Response
        """
        service_name = 'executor' + str(executor_id) + '/load'
        load_client = LoadNodeClient(service_name)
        executor_response = load_client.send_request(name)
        load_client.destroy_node()
        return executor_response

def main(args=None):
    rclpy.init()
    commander = CommanderNode()

    rclpy.spin(commander)

    commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()