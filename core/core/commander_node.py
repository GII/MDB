import os
import rclpy
import yaml
import random

from rclpy.node import Node
from core.config import saved_data_dir

from std_msgs.msg import String
from core.service_client import ServiceClient

from core_interfaces.srv import AddExecutionNode, DeleteExecutionNode, MoveCognitiveNodeToExecutionNode
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

        Creates a ROS 2 node named 'commander node' and a service for the User to send commands.
        """
        
        super().__init__('commander')

        self.last_id = 0
        self.executor_ids = []
        self.nodes = {}

        for executor_id in self.executor_ids:
            self.nodes[executor_id] = []
            
        # Add Execution Node Service for the Execution Nodes
        self.add_execution_node_service = self.create_service(
            AddExecutionNode,
            'commander/add_executor',
            self.add_execution_node
        )

        # Delete Execution Node Service for the User
        self.delete_execution_node_service = self.create_service(
            DeleteExecutionNode,
            'commander/delete_executor',
            self.delete_execution_node
        )

        # Move Cognitive Node Service for the User
        self.move_cognitive_node_service = self.create_service(
            MoveCognitiveNodeToExecutionNode,
            'commander/move_cognitive_node_to',
            self.move_cognitive_node_to
        )

        # Create Node Service for the User
        self.create_node_service = self.create_service(
            CreateNode,
            'commander/create',
            self.create_node
        )
                
        # Read Node Service for the User
        self.read_node_service = self.create_service(
            ReadNode,
            'commander/read',
            self.read_node
        )
                
        # Delete Node Service for the User
        self.delete_node_service = self.create_service(
            DeleteNode,
            'commander/delete',
            self.delete_node
        )
                
        # Save Node Service for the User
        self.save_node_service = self.create_service(
            SaveNode,
            'commander/save',
            self.save_node
        )

        # Load Node Service for the User
        self.load_node_service = self.create_service(
            LoadNode,
            'commander/load',
            self.load_node
        )

        # Load Config Service for the User
        self.load_config_service = self.create_service(
            LoadConfig,
            'commander/load_config',
            self.load_experiment
        )

        # Save Config Service for the User
        self.save_config_service = self.create_service(
            SaveConfig,
            'commander/save_config',
            self.save_config
        )

        # Stop Execution Service for the User
        self.stop_execution_service = self.create_service(
            StopExecution,
            'commander/stop_execution',
            self.stop_execution
        )

        # Stop Execution Topic
        self.stop_execution_node_publisher = self.create_publisher(
           String,
           'stop_execution_node',
           10 
        )

    def add_execution_node(self, request, response):
        """
        Adds a new execution node to the system.

        This method increments the last used ID, creates a new execution node with the updated ID,
        and returns the assigned ID in the response.

        :param request: The request to add a new execution node.
        :type request: core_interfaces.srv.AddExecutionNode_Request
        :return: The response with the assigned ID for the new execution node.
        :rtype: core_interfaces.srv.AddExecutionNode_Response
        """        
        self.last_id += 1
        new_id = str(self.last_id)
        
        self.get_logger().info(f"Adding new execution node with id {new_id}.")

        self.executor_ids.append(new_id)
        self.nodes[new_id] = []
        
        response.id = str(new_id)
        return response

    def delete_execution_node(self, request, response):
        """
        Deletes an execution node and redistributes its cognitive nodes to other executors.

        This method performs the following steps:
        1. Saves all cognitive nodes of the specified execution node.
        2. Deletes the execution node and stops its execution.
        3. Loads the saved cognitive nodes into other executors.

        :param request: The request to delete an execution node.
        :type request: core_interfaces.srv.DeleteExecutionNode_Request
        :return: The response indicating whether the deletion was successful.
        :rtype: core_interfaces.srv.DeleteExecutionNode_Response
        """        
        ex_id = request.id

        # save all cognitive nodes
        self.get_logger().info(f"Saving all the nodes of the executor {ex_id}...")
        saved = self.send_save_all_nodes_request_to_executor(ex_id)

        if saved:
            # delete the execution node
            self.get_logger().info(f'Deleting executor: {ex_id}...')
            self.send_stop_request_to_executor(ex_id)
            del self.nodes[ex_id]
            self.executor_ids.remove(ex_id)

            # load the cognitive nodes in another executor

            folder_name = 'execution_node_' + str(ex_id) + '_data'
            executor_folder_path = os.path.join(saved_data_dir, folder_name)     
                
            if not os.path.exists(executor_folder_path):
                self.get_logger().info(f'Executor folder {executor_folder_path} not found.')
        
            else:

                self.get_logger().info(f'Loading nodes...')
                for filename in os.listdir(executor_folder_path):
                    if filename.endswith(".yaml"):
                        node_file_path = os.path.join(executor_folder_path, filename)
                        
                        node_name, _ = os.path.splitext(filename)

                        if self.node_exists(node_name):
                            self.get_logger().info(f'Node {node_name} already exists.')

                        ex = self.get_lowest_load_executor()

                        executor_response = self.send_load_request_to_executor(ex, node_name, node_file_path)

                        self.register_node(ex, node_name)

                        if executor_response.loaded:
                            self.get_logger().info(f'Node {node_name} loaded in executor {ex}.')

            response.deleted = True

        else:
            self.get_logger().info(f"Could not delete executor {ex_id}.")
            response.deleted = False
        
        return response
    
    def move_cognitive_node_to(self, request, response):
        """
        Moves a cognitive node from its current executor to a specified executor.

        This method performs the following steps:
        1. Checks if the specified node exists.
        2. Saves the data of the node.
        3. Removes the node from its current executor.
        4. Loads the node into the specified executor.

        :param request: The request to move a cognitive node.
        :type request: core_interfaces.srv.MoveCognitiveNodeToExecutionNode_Request
        :return: The response indicating whether the movement was successful.
        :rtype: core_interfaces.srv.MoveCognitiveNodeToExecutionNode_Response
        """        
        ex_id = request.ex_id
        node_name = request.name
        self.get_logger().info(f"Moving node {node_name} to executor {ex_id}...")

        if self.node_exists(node_name):

            current_ex_id = self.get_executor_for_node(node_name)

            # save node data

            self.send_save_request_to_executor(current_ex_id, node_name)
            
            # remove node from current executor
            self.send_delete_request_to_executor(current_ex_id, node_name)

            # load node in new executor
            file_path = os.path.join(saved_data_dir, node_name + '.yaml')

            self.send_load_request_to_executor(ex_id, node_name, file_path)

            self.get_logger().info(f"Node {node_name} moved to executor {ex_id}.")
            response.moved = True

        else:
            self.get_logger().info(f"Node {node_name} not found.")
            response.moved = False


        response.moved = True
        return response

    
    def create_node(self, request, response):
        """
        Handle the creation of a cognitive node.

        This method performs the following steps:
        1. Extracts information from the creation request, including node name, class name, and parameters.
        2. Checks if the specified node already exists.
        3. Determines the executor with the lowest load to distribute the creation request.
        4. Sends a create request to the selected executor to create the cognitive node.
        5. Registers the created node in the system.

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

        This method performs the following steps:
        1. Extracts information the node name from the read request.
        2. Checks if the specified node exists.
        3. Retrieves the executor associated with the node.
        4. Sends a read request to the executor to obtain data from the cognitive node.
        5. Parses the received data and logs information.
        6. Sets the response data with the information obtained from the cognitive node.

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

        This method performs the following steps:
        1. Extracts the node name from the delete request.
        2. Checks if the specified node exists.
        3. Retrieves the executor associated with the node.
        4. Sends a delete request to the executor to remove the cognitive node.
        5. Removes the node from the local data structures.
        
        :param request: The delete request.
        :type request: core_interfaces.srv.DeleteNode_Request
        :param response: The response to the delete request.
        :type response: core_interfaces.srv.DeleteNode_Response
        :return: The response to the delete request.
        :rtype: core_interfaces.srv.DeleteNode_Response
        """        
        name = str(request.name)
        self.get_logger().info(f"Deleting node {name}...")
        
        if not self.node_exists(name):
            self.get_logger().info(f"Node {name} doesn't exist.")
            response.deleted = False
        
        else:
            ex = self.get_executor_for_node(name)

            executor_response = self.send_delete_request_to_executor(ex, name)
          
            self.remove_node_from_executor(ex, name)
            self.get_logger().info(f"Node {name} deleted from executor {ex}")

            response = executor_response

        return response

    def save_node(self, request, response):
        """
        Handle saving the state of a cognitive node.

        This method performs the following steps:
        1. Checks if the specified node exists.
        2. Determines the current executor of the node.
        3. Sends a save request to the executor to save the state of the node.

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

        This method performs the following steps:
        1. Extracts information from the load request, including the node name and file path.
        2. Checks if the specified node already exists.
        3. Retrieves the executor with the lowest load.
        4. Sends a load request to the executor to load the cognitive node from the specified file.
        5. Registers the loaded node in the local data structures.

        :param request: The load request.
        :type request: core_interfaces.srv.LoadNode_Request
        :param response: The response to the load request.
        :type response: core_interfaces.srv.LoadNode_Response
        :return: The response to the load request.
        :rtype: core_interfaces.srv.LoadNode_Response
        """
        name = str(request.name)
        file_path = str(request.file)

        if self.node_exists(name):
            self.get_logger().info(f'Node {name} already exists.')
            response.loaded = False
        
        else:
            
            ex = self.get_lowest_load_executor()
            
            executor_response = self.send_load_request_to_executor(ex, name, file_path)
            
            self.register_node(ex, name)
            
            if executor_response.loaded:
                self.get_logger().info(f'Node {name} loaded in executor {ex}.')

            response = executor_response
                    
        return response
    
    def load_experiment(self, request, response):
        """
        Load an experiment from a file.

        This method performs the following steps:
        1. Extracts the experiment file path from the load request.
        2. Checks if the specified experiment file exists.
        3. Reads the YAML data from the experiment file.
        4. Retrieves information about cognitive nodes from the loaded data.
        5. Iterates over the cognitive nodes, creating and registering them in the system.

        :param request: The load experiment request.
        :type request: core_interfaces.srv.LoadConfig_Request
        :param response: The response to the load experiment request.
        :type response: core_interfaces.srv.LoadConfig_Response
        :return: The response to the load experiment request.
        :rtype: core_interfaces.srv.LoadConfig_Response
        """    
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

            if data.get('Experiment'):
                experiment_data=data['Experiment']
                name=experiment_data['name']
                class_name=experiment_data['class_name']
                if experiment_data.get('parameters'):
                    params_dict=experiment_data['parameters']
                    params_dict['LTM_id']='ltm_0' #TODO Handle multiple LTMs
                    parameters=str(params_dict)

                self.get_logger().info(f"Loading {class_name} {name}...")

                ex= self.get_lowest_load_executor()

                executor_response= self.send_create_request_to_executor(ex, name, class_name, parameters)
                self.register_node(ex, name)
                self.get_logger().info(f'Process {name} created in executor {ex}.')

            response.loaded = True
                   
        return response
      
    def save_config(self, request, response):
        """
        Saves the state of the system to a YAML file.

        This method performs the following steps:
        1. Constructs the file path using the saved data directory and the specified file name.
        2. Prepares the data structure to be saved, including the LTM nodes organized by type.
        3. Iterates over each execution node to retrieve the data of all cognitive nodes.
        4. Appends the retrieved node data to the corresponding type in the prepared data structure.
        5. Writes the final data structure to a YAML file.

        :param request: The save config request.
        :type request: core_interfaces.srv.SaveConfig_Request
        :param response: The response to the save config request.
        :type response: core_interfaces.srv.SaveConfig_Response
        :return: The response to the save config request.
        :rtype: core_interfaces.srv.SaveConfig_Response
        """        
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
        """
        Stop the execution of all execution nodes in the system.

        This method performs the following steps:
        1. Iterates over each execution node to send a stop request and stop its execution.
        2. Clears the list of nodes associated with each execution node.
        3. Publishes a stop message for each execution node.

        :param request: The stop execution request.
        :type request: core_interfaces.srv.StopExecution_Request
        :param response: The response to the stop execution request.
        :type response: core_interfaces.srv.StopExecution_Response
        :return: The response to the stop execution request.
        :rtype: core_interfaces.srv.StopExecution_Response
        """
        self.get_logger().info(f'Stopping execution...')

        for ex in self.executor_ids:
            executor_response = self.send_stop_request_to_executor(ex)
            self.nodes[ex] = []
            stop_msg = String()
            stop_msg.data = str(ex)
            self.stop_execution_node_publisher.publish(stop_msg)
            self.get_logger().info(f'Execution of execution node {ex} stopped.')
        
        self.get_logger().info(f'Execution stopped.')
        return executor_response

    # TODO: implement this method with load balancing
    def get_lowest_load_executor(self):
        """
        Get the executor with the lowest load.

        :return: The executor with the lowest load.
        :rtype: int
        """
        pos = random.randint(0, self.executor_ids.__len__() -1)

        ex = self.executor_ids[pos]
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

    def send_delete_request_to_executor(self, executor_id, name):
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
        executor_response = delete_client.send_request(name=name)
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

    def send_load_request_to_executor(self, executor_id, name, file_path):
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
        executor_response = load_client.send_request(name=name, file=file_path)
        load_client.destroy_node()
        return executor_response

    def send_read_all_nodes_request_to_executor(self, executor_id):
        """
        Send a request to read data from all cognitive nodes in a specific execution node.
        :param executor_id: The identifier of the target execution node.
        :type executor_id:  str
        :return: The response containing data from all cognitive nodes.
        :rtype: core_interfaces.srv.ReadNode_Response
        """
        service_name = 'execution_node_' + str(executor_id) + '/read_all_nodes'
        read_all_nodes_client = ServiceClient(ReadNode, service_name)
        executor_response = read_all_nodes_client.send_request()
        read_all_nodes_client.destroy_node()
        return executor_response
    
    def send_save_all_nodes_request_to_executor(self, executor_id):
        """
        Send a request to save the state of all cognitive nodes in a specific execution node.

        :param executor_id: The identifier of the target execution node.
        :type executor_id: str
        :return: The response indicating the success of saving all cognitive nodes.
        :rtype: core_interfaces.srv.SaveNode_Response
        """        
        service_name = 'execution_node_' + str(executor_id) + '/save_all_nodes'
        save_all_nodes_client = ServiceClient(SaveNode, service_name)
        executor_response = save_all_nodes_client.send_request()
        save_all_nodes_client.destroy_node()
        return executor_response 
       
    def send_stop_request_to_executor(self, executor_id):
        """
        Send a request to stop the execution of all cognitive nodes in a specific execution node.

        :param executor_id: The identifier of the target execution node.
        :type executor_id: str
        :return: The response indicating the success of stopping the execution.
        :rtype: core_interfaces.srv.StopExecution_Response
        """        
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