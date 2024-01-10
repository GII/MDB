import os
import sys
import rclpy
import yaml
import random
from rclpy.node import Node
from core.config import saved_data_dir

from core.service_client import ServiceClient
from core_interfaces.srv import CreateNode, ReadNode, DeleteNode, SaveNode, LoadNode
from core_interfaces.srv import SaveConfig, LoadConfig, StopExecution

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
        super().__init__('commander')
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

        # Load Config Service for the user
        self.load_config_service = self.create_service(
            LoadConfig,
            'commander/load_config',
            self.load_experiment
        )

        # Save Config Service for the user
        self.save_config_service = self.create_service(
            SaveConfig,
            'commander/save_config',
            self.save_config
        )

        # Stop Execution Service for the user
        self.stop_execution_service = self.create_service(
            StopExecution,
            'commander/stop_execution',
            self.stop_execution
        )

    def create_node(self, request, response):
        """
        Handle the creation of a cognitive node.

        :param request: The creation request.
        :type request: core_interfaces.srv.CreateNode_Request
        :param response: The response to the creation request.
        :type response: core_interfaces.srv.CreateNode_Response
        :return: The response to the creation request.
        :rtype: core_interfaces.srv.CreateNode_Response
        """
        name = str(request.name)
        class_name = str(request.class_name)
        parameters = str(request.parameters)

        self.get_logger().info(f'Creating new {class_name} {name}...')
        
        if self.node_exists(name):
            self.get_logger().info(f'Node {name} already exists.')
            response.created = False
        
        else:
           
            ex = self.get_lowest_load_executor()
            
            executor_response = self.send_create_request_to_executor(ex, name, class_name, parameters)
            
            self.register_node(ex, name)
            
            self.get_logger().info(f'Node {name} created in executor {ex}.')

            response = executor_response
        return response

    def read_node(self, request, response):
        """
        Handle reading data from a cognitive node.

        :param request: The read request.
        :type request: core_interfaces.srv.ReadNode_Request
        :param response: The response to the read request.
        :type response: core_interfaces.srv.ReadNode_Response
        :return: The response to the read request.
        :rtype: core_interfaces.srv.ReadNode_Response
        """        
        name = str(request.name)
        
        self.get_logger().info(f'Reading node {name}...')

        if not self.node_exists(name):
            response.data = ''
            self.get_logger().info(f'Node {name} does not exist.')
        
        else:
            ex = self.get_executor_for_node(name)

            executor_response = self.send_read_request_to_executor(ex, name)

            node_data = yaml.load(executor_response.data, Loader=yaml.FullLoader)

            self.get_logger().info(f'Read node {name}: {node_data}.')

            response = executor_response
        return response

    def delete_node(self, request, response):
        """
        Handle the deletion of a cognitive node.

        :param request: The delete request.
        :type request: core_interfaces.srv.DeleteNode_Request
        :param response: The response to the delete request.
        :type response: core_interfaces.srv.DeleteNode_Response
        :return: The response to the delete request.
        :rtype: core_interfaces.srv.DeleteNode_Response
        """        
        name = str(request.name)
        class_name = str(request.class_name)
        self.get_logger().info(f"Deleting node {name}...")
        
        if not self.node_exists(name):
            self.get_logger().info(f"Node {name} doesn't exist.")
            response.deleted = False
        
        else:
            class_name = str(request.class_name)

            ex = self.get_executor_for_node(name)

            executor_response = self.send_delete_request_to_executor(ex, name, class_name)
          
            self.remove_node_from_executor(ex, name)
            self.get_logger().info(f"Node {name} deleted from executor {ex}")

            response = executor_response

        return response

    def save_node(self, request, response):
        """
        Handle saving the state of a cognitive node.

        :param request: The save request.
        :type request: core_interfaces.srv.SaveNode_Request
        :param response: The response to the save request.
        :type response: core_interfaces.srv.SaveNode_Response
        :return: The response to the save request.
        :rtype: core_interfaces.srv.SaveNode_Response
        """        
        name = str(request.name)
                
        if not self.node_exists(name):
            self.get_logger().info(f'Node {name} does not exist.')
            response.saved = False
        
        else:
            ex = self.get_executor_for_node(name)

            executor_response = self.send_save_request_to_executor(ex, name)
            if executor_response.saved:
                self.get_logger().info(f'Node {name} saved.')

            response = executor_response

        return response

    def load_node(self, request, response):
        """
        Handle loading a cognitive node from a configuration.

        :param request: The load request.
        :type request: core_interfaces.srv.LoadNode_Request
        :param response: The response to the load request.
        :type response: core_interfaces.srv.LoadNode_Response
        :return: The response to the load request.
        :rtype: core_interfaces.srv.LoadNode_Response
        """
        name = str(request.name)

        if self.node_exists(name):
            self.get_logger().info(f'Node {name} already exists.')
            response.loaded = False
        
        else:
            
            ex = self.get_lowest_load_executor()
            
            executor_response = self.send_load_request_to_executor(ex, name)
            
            self.register_node(ex, name)
            
            if executor_response.loaded:
                self.get_logger().info(f'Node {name} loaded in executor {ex}.')

            response = executor_response
                    
        return response
    
    def load_experiment(self, request, response):

        experiment_file = str(request.file)

        if not os.path.exists(experiment_file):

            self.get_logger().info(f"Couldn't load experiment. File {experiment_file} not found")
            response.loaded = False
        
        else:
            
            with open(experiment_file, 'r') as file:
                data = yaml.load(file, Loader=yaml.FullLoader)
            self.get_logger().info(f'Loading file {experiment_file}')

            nodes = data['LTM']['Nodes']
            
            for class_name, node_list in nodes.items():
                for node in node_list:
                    name = node['name']
                    class_name = node['class_name']
                    if node.get('parameters'):
                        parameters = str(node['parameters'])
                    else:
                        parameters = ''
                    self.get_logger().info(f"Loading {class_name} {name}...")

                    if self.node_exists(name):
                        self.get_logger().info(f"Node {name} already exists.")

                    else:
                    
                        ex = self.get_lowest_load_executor()
                        
                        executor_response = self.send_create_request_to_executor(ex, name, class_name, parameters)
                        
                        self.register_node(ex, name)
                        
                        self.get_logger().info(f'Node {name} created in executor {ex}.')

            response.loaded = True
                   
        return response
      
    def save_config(self, request, response):
        response.saved = True
        
        file_name = str(request.file)
        file_path = os.path.join(saved_data_dir, file_name + '.yaml')

        data_to_save = {}
        nodes_dict = {'Perception': [], 'Need': [], 'Goal': [], 'Drive': [], 'UtilityModel': [], 'WorldModel': [], 'Policy': [], 'PNode': []}
        data_to_save['LTM'] = {'Nodes': nodes_dict}

        self.get_logger().info(f'Saving config to {file_path}')

        node_list = []

        for ex in self.executor_ids:
            self.get_logger().info(f'Reading data from execution node {ex}')
            executor_response = self.send_read_all_nodes_request_to_executor(ex)
            self.get_logger().info(f'Data from execution node {ex} read.')

            node_list.extend(yaml.load(executor_response.data, Loader=yaml.FullLoader))

        self.get_logger().info(f'Node list: {node_list}')

        for node in node_list:
            data_to_save['LTM']['Nodes'][node['node_type']].append(node)
      
        with open(file_path, 'w') as file:
            yaml.dump(data_to_save, file)

        return response

    def stop_execution(self, request, response):
        
        self.get_logger().info(f'Stopping execution...')

        for ex in self.executor_ids:
            self.get_logger().info(f'Stopping execution from execution node {ex}')
            executor_response = self.send_stop_request_to_executor(ex)
            self.nodes[ex] = []
            self.get_logger().info(f'Execution from execution node {ex} stopped.')
        
        self.get_logger().info(f'Execution stopped.')

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

    def send_create_request_to_executor(self, executor_id, name, class_name, parameters):
        """
        Send a 'create' request to an executor.

        :param executor_id: The ID of the executor.
        :type executor_id: int
        :param name: The name of the node to create.
        :type name: str
        :param class_name: The type of the node to create.
        :type class_name: str
        :return: The response from the executor.
        :rtype: core_interfaces.srv.CreateNode_Response
        """
        service_name = 'execution_node_' + str(executor_id) + '/create'
        create_client = ServiceClient(CreateNode, service_name)
        executor_response = create_client.send_request(name=name, class_name=class_name, parameters=parameters)       
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
        :rtype: core_interfaces.srv.ReadNode_Response
        """
        service_name = 'execution_node_' + str(executor_id) + '/read'
        read_client = ServiceClient(ReadNode, service_name)
        executor_response = read_client.send_request(name=name)
        read_client.destroy_node()
        return executor_response

    def send_delete_request_to_executor(self, executor_id, name, class_name):
        """
        Send a 'delete' request to an executor.

        :param executor_id: The ID of the executor.
        :type executor_id: int
        :param name: The name of the node to delete.
        :type name: str
        :param class_name: The type of the node to delete.
        :type class_name: str
        :return: The response from the executor.
        :rtype: core_interfaces.srv.DeleteNode_Response
        """
        service_name = 'execution_node_' + str(executor_id) + '/delete'
        delete_client = ServiceClient(DeleteNode, service_name)
        executor_response = delete_client.send_request(name=name, class_name=class_name)
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
        :rtype: core_interfaces.srv.SaveNode_Response
        """
        service_name = 'execution_node_' + str(executor_id) + '/save'
        save_client = ServiceClient(SaveNode, service_name)
        executor_response = save_client.send_request(name=name)
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
        :rtype: core_interfaces.srv.LoadNode_Response
        """
        service_name = 'execution_node_' + str(executor_id) + '/load'
        load_client = ServiceClient(LoadNode, service_name)
        executor_response = load_client.send_request(name=name)
        load_client.destroy_node()
        return executor_response

    def send_read_all_nodes_request_to_executor(self, executor_id):
        service_name = 'execution_node_' + str(executor_id) + '/read_all_nodes'
        read_all_nodes_client = ServiceClient(ReadNode, service_name)
        executor_response = read_all_nodes_client.send_request()
        read_all_nodes_client.destroy_node()
        return executor_response
    
    def send_stop_request_to_executor(self, executor_id):
        service_name = 'execution_node_' + str(executor_id) + '/stop_execution'
        stop_execution_client = ServiceClient(StopExecution, service_name)
        executor_response = stop_execution_client.send_request()
        stop_execution_client.destroy_node()
        return executor_response

def main(args=None):
    rclpy.init()
    commander = CommanderNode()

    rclpy.spin(commander)

    commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()