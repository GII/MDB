import os
import sys
import rclpy
import yaml
import importlib

from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
# from rclpy.executors import MultiThreadedExecutor

from mdb_interfaces.srv import CreateNode, ReadNode, DeleteNode, SaveNode, LoadNode

from mdb.config import saved_data_dir
from mdb.utils import class_from_classname

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
        
        # # Send To Executor Service for the commander node
        # self.send_to_executor_service = self.create_service(
        #     SendToExecutor,
        #     'send_to_executor_' + str(self.id),
        #     self.handle_command
        # )
                
        # Create Node Service for the commander node
        self.create_node_service = self.create_service(
            CreateNode,
            'executor' + str(self.id) + '/create',
            self.create_node
        )
                
        # Read Node Service for the commander node
        self.read_node_service = self.create_service(
            ReadNode,
            'executor' + str(self.id) + '/read',
            self.read_node
        )
                
        # Delete Node Service for the commander node
        self.delete_node_service = self.create_service(
            DeleteNode,
            'executor' + str(self.id) + '/delete',
            self.delete_node
        )
                
        # Save Node Service for the commander node
        self.save_node_service = self.create_service(
            SaveNode,
            'executor' + str(self.id) + '/save',
            self.save_node
        )

        # Load Node Service for the commander node
        self.load_node_service = self.create_service(
            LoadNode,
            'executor' + str(self.id) + '/load',
            self.load_node
        )
    
    def create_node(self, request, response):
        """
        Create a new cognitive node.
        If the node doesn't have previous data, it is created with the default values.
        In other case, the existent data is loaded.

        :param name: The name of the node to be created.
        :type name: str
        :param node_type: The type of the node to be created.
        :type node_type: str
        :param data: Optional data to initialize the new node.
        :type data: dict
        """

        node_type = str(request.node_type)
        name = str(request.name)
        
        self.get_logger().info('Creating new ' + str(node_type) + ' ' + str(name) + '...')

        new_node = class_from_classname(node_type)(name)

        self.nodes[name] = new_node
        self.executor.add_node(new_node)

        self.get_logger().info('Added node: ' + str(new_node.get_name()))
        response.created = True
        return response

    def read_node(self, request, response):
        """
        Retrieve a node by its name.

        :param name: The name of the node to retrieve.
        :type name: str
        :return: The node with the specified name, or None if not found.
        :rtype: mdb.cognitive_node.CognitiveNode or None
        """
        name = str(request.name)
        
        self.get_logger().info('Reading node: ' + name + '...')

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

        self.get_logger().info('Deleting node: ' + name + '...')

        if name in self.nodes:
            node_to_delete = self.nodes.pop(name)
            self.executor.remove_node(node_to_delete)
            node_to_delete.destroy_node()
            self.get_logger().info('Deleted node: ' + name + '.')
            response.deleted = True
        else:
            self.get_logger().info('Node with name ' + name + ' not found.')
            response.deleted = False
        return response

    def save_node(self, request, response):
        """
        Save the state of a node to a YAML file.

        :param name: The name of the node to save.
        :type name: str
        """

        name = str(request.name)
        
        self.get_logger().info('Saving node: ' + name + '...')

        node_to_save = self.nodes.get(name)
        
        if node_to_save is not None:
            state_file = os.path.join(saved_data_dir, name + '.yaml')
            data_to_save = node_to_save.get_data()
            with open(state_file, 'w') as file:
                yaml.dump(data_to_save, file)
            self.get_logger().info('Saved node: ' + name + '.')
            response.saved = True
        else:
            self.get_logger().info('Node with name ' + name + ' not found.')         
            response.saved = False
        return response

    def load_node(self, request, response): # TODO: Check that it doesn't already exists a node with that name
        """
        Load the state of a node from a YAML file and create the node.

        :param name: The name of the node to load.
        :type name: str
        :param node_type: The type of the node to be created.
        :type node_type: str
        """        
        
        name = str(request.name)

        self.get_logger().info('Loading node: ' + name + '...')

        state_file = os.path.join(saved_data_dir, name + '.yaml')
        if os.path.exists(state_file):
            with open(state_file, 'r') as file:
                data = yaml.load(file, Loader=yaml.FullLoader)
            
            node_type = data['node_type']
            del data['node_type']
            
            loaded_node = class_from_classname(node_type)(**data)

            self.nodes[name] = loaded_node
            self.executor.add_node(loaded_node)

            self.get_logger().info('Loaded node: ' + str(name))
            response.loaded = True
        return response

    # def handle_command(self, request, response):
    #     """
    #     Handle command requests.

    #     :param request: The command request.
    #     :type request: mdb_interfaces.srv.SendToExecutor_Request
    #     :param response: The response to the command.
    #     :type response: mdb_interfaces.srv.SendToExecutor_Response
    #     :return: The response to the command.
    #     :rtype: mdb_interfaces.srv.SendToExecutor_Response
    #     """        
    #     command = str(request.command)
    #     name = str(request.name)
            
    #     self.get_logger().info('Received request: ' + command)
        
    #     if(command == 'create'):
    #         type = str(request.type)
    #         self.create_node(name, type)
    #         response.msg = 'Node created: ' + name
        
    #     elif (command == 'read'):
    #         read_node = self.read_node(name)
    #         if read_node is not None:
    #             response.msg = str(read_node)
    #         else:
    #             response.msg = 'Couldnt read node ' + name + '.'

    #     elif (command == 'delete'):
    #         self.delete_node(name)
    #         response.msg = 'Node deleted: ' + name

    #     elif (command == 'save'):
    #         self.save_node(name)
    #         response.msg = 'Node saved: ' + name

    #     elif (command == 'load'):
    #         self.load_node(name)
    #         response.msg = 'Node loaded: ' + name
    #     else:
    #         self.get_logger().info('Wrong command.')
    #         response.msg = 'Wrong request: ' + command
    #     return response

    
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