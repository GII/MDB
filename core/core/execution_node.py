import os
import sys
import rclpy
import yaml
import importlib

from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
# from rclpy.executors import MultiThreadedExecutor

from core_interfaces.srv import CreateNode, ReadNode, DeleteNode, SaveNode, LoadNode, SaveConfig
from core.service_client import ServiceClient

from core.config import saved_data_dir
from core.utils import class_from_classname

class ExecutionNode(Node):
    """
    This class represents an execution node, which can execute several cognitive nodes.
    It is subscribed to the topic where the commander node sends commands.
    """

    # TODO: Transform all the IDs into strings

    def __init__(self, executor, id):
        """
        Constructor for the ExecutionNode class.

        :param executor: The ROS2 executor for the node.
        :type executor: rclpy.executors.SingleThreadedExecutor
        :param id: The identifier of the execution node.
        :type id: int
        """
        
        super().__init__('execution_node_' + str(id))
        self.get_logger().info('Creating execution node')
        
        self.id = id
        self.nodes = {}
        self.executor = executor
                
        # Create Node Service for the commander node
        self.create_node_service = self.create_service(
            CreateNode,
            'execution_node_' + str(self.id) + '/create',
            self.create_node
        )
                
        # Read Node Service for the commander node
        self.read_node_service = self.create_service(
            ReadNode,
            'execution_node_' + str(self.id) + '/read',
            self.read_node
        )
                
        # Delete Node Service for the commander node
        self.delete_node_service = self.create_service(
            DeleteNode,
            'execution_node_' + str(self.id) + '/delete',
            self.delete_node
        )
                
        # Save Node Service for the commander node
        self.save_node_service = self.create_service(
            SaveNode,
            'execution_node_' + str(self.id) + '/save',
            self.save_node
        )

        # Load Node Service for the commander node
        self.load_node_service = self.create_service(
            LoadNode,
            'execution_node_' + str(self.id) + '/load',
            self.load_node
        )

        # Read All Nodes service for the commander node
        self.read_all_nodes = self.create_service(
            ReadNode,
            'execution_node_' + str(self.id) + '/read_all_nodes',
            self.read_all_nodes
        )
    
    def create_node(self, request, response):
        """
        Create a new cognitive node.
        If the node doesn't have previous data, it is created with the default values.
        In other case, the existent data is loaded.

        :param name: The name of the node to be created.
        :type name: str
        :param class_name: The type of the node to be created.
        :type class_name: str
        :param data: Optional data to initialize the new node.
        :type data: dict
        """

        class_name = str(request.class_name)
        name = str(request.name)
        yaml_parameters = str(request.parameters)
        if yaml_parameters:
            parameters = yaml.safe_load(yaml_parameters)
        else:
            parameters = {}
      
        self.get_logger().info(f'Creating new {class_name} {name}...')

        new_node = class_from_classname(class_name)(name, **parameters)

        self.nodes[name] = new_node
        self.executor.add_node(new_node)

        self.get_logger().info(f'Added node: {name}.')
        response.created = True
        return response

    def read_node(self, request, response):
        """
        Retrieve a node by its name.

        :param name: The name of the node to retrieve.
        :type name: str
        :return: The node with the specified name, or None if not found.
        :rtype: core.cognitive_node.CognitiveNode or None
        """
        name = str(request.name)
        
        self.get_logger().info(f'Reading node: {name}...')

        if name in self.nodes:
            response.data = str(self.nodes.get(name))
        else:
            response.data = ''
        return response

    def delete_node(self, request, response):
        """
        Delete a node by name.

        :param name: The name of the node to delete.
        :type name: str
        """
        name = str(request.name)

        self.get_logger().info(f'Deleting node: {name}...')

        if name in self.nodes:
            node_to_delete = self.nodes.pop(name)
            self.executor.remove_node(node_to_delete)
            # node_to_delete.delete_from_LTM()
            node_to_delete.destroy_node()
            self.get_logger().info(f'Deleted node: {name}.')
            response.deleted = True
        else:
            self.get_logger().info(f'Node {name} not found.')
            response.deleted = False
        return response

    def save_node(self, request, response):
        """
        Save the state of a node to a YAML file.

        :param name: The name of the node to save.
        :type name: str
        """

        name = str(request.name)
        
        self.get_logger().info(f'Saving node: {name} ...')

        node_to_save = self.nodes.get(name)
        
        if node_to_save is not None:
            state_file = os.path.join(saved_data_dir, name + '.yaml')
            data_to_save = node_to_save.get_data()
            with open(state_file, 'w') as file:
                yaml.dump(data_to_save, file)
            self.get_logger().info(f'Saved node {name}.')
            response.saved = True
        else:
            self.get_logger().info(f'Node {name} not found.')         
            response.saved = False
        return response

    def load_node(self, request, response):
        """
        Load the state of a node from a YAML file and create the node.

        :param name: The name of the node to load.
        :type name: str
        :param class_name: The type of the node to be created.
        :type class_name: str
        """        
        
        name = str(request.name)
        response.loaded = False

        node_to_load = self.nodes.get(name)
        
        if node_to_load is None:

            self.get_logger().info(f'Loading node: {name} ...')

            state_file = os.path.join(saved_data_dir, name + '.yaml')
            if os.path.exists(state_file):
                with open(state_file, 'r') as file:
                    data = yaml.load(file, Loader=yaml.FullLoader)
                
                class_name = data['class_name']
                del data['node_type']
                
                loaded_node = class_from_classname(class_name)(**data)

                self.nodes[name] = loaded_node
                self.executor.add_node(loaded_node)

                self.get_logger().info(f'Loaded node: {name}')
                response.loaded = True
            else:
                self.get_logger().info(f"File {state_file} not found. Couldn't load node.")
        else:
            self.get_logger().info(f"Node {name} already exists. Couldn't load node.")

        return response
    
    def read_all_nodes(self, _, response):
        """
        Returns a list of nodes with the data of all the nodes in this execution node.
        """

        self.get_logger().info(f'Reading all the nodes from execution node {self.id}')

        nodes = []

        for name in self.nodes:
            
            node = self.nodes.get(name)
            nodes.append(node.get_data())

        response.data = str(nodes)

        return response
    
# single threaded executor
def main(args=None):
    rclpy.init(args=args)
    id = int(sys.argv[1])
    executor = SingleThreadedExecutor() # TODO: TBD if it is single or multi threaded executor.
    execution_node = ExecutionNode(executor, id)
    executor.add_node(execution_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    for node in execution_node.nodes.values():
        executor.remove_node(node)
        node.destroy_node()

# # multi threaded executor
# def main(args=None):
#     rclpy.init(args=args)
#     id = int(sys.argv[1])

#     executor = MultiThreadedExecutor(num_threads=2)
#     execution_node = ExecutionNode(executor, id)
    
#     executor.add_node(execution_node)

#     try:
#         executor.spin()
#     except KeyboardInterrupt:
#         pass

#     for name, node in execution_node.nodes.items():
#         executor.remove_node(node)
#         node.destroy_node()


if __name__ == '__main__':
    main()