import os
import sys
import rclpy
import yaml
from rclpy.node import Node

from mdb.a_node import ANode
from rclpy.executors import SingleThreadedExecutor
# from rclpy.executors import MultiThreadedExecutor
from mdb_interfaces.srv import SendCommand
from mdb_interfaces.msg import Command

data_dir = '/home/cristina/ros2_ws/src/mdb/saved_data' # TODO: Cambiar a ruta relativa

class ExecutionNode(Node):
    """
    This class represents an execution node, which can execute several cognitive nodes.
    It is subscribed to the topic where the commander node sends commands.
    """

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
        
        # Send command service
        self.send_command_service = self.create_service(
            SendCommand,
            'send_command_' + str(self.id),
            self.send_command
        )

    def create_node(self, name, data=None): # TODO: add different node types
        """
        Create a new node.

        :param name: The name of the node to be created.
        :type name: str
        :param data: Optional data to initialize the new node.
        :type data: dict
        """
        if data == None: # TODO: differenciate node types
            
            new_node = ANode(name)
        else:
            new_node = ANode(name, **data)

        self.nodes[name] = new_node
        self.executor.add_node(new_node)
        self.get_logger().info('Added node: ' + str(new_node.get_name()))

    def read_node(self, name):
        """
        Retrieve a node by its name.

        :param name: The name of the node to retrieve.
        :type name: str
        :return: The node with the specified name, or None if not found.
        :rtype: mdb.cognitive_node.CognitiveNode or None
        """
        if name in self.nodes:
            return self.nodes.get(name)
        else:
            return None

    def delete_node(self, name):
        """
        Delete a node by name.

        :param name: The name of the node to delete.
        :type name: str
        """
        if name in self.nodes:
            node_to_delete = self.nodes.pop(name)
            self.executor.remove_node(node_to_delete)
            node_to_delete.destroy_node()
            self.get_logger().info('Deleted node: ' + name + '.')
        else:
            self.get_logger().info('Node with name ' + name + ' not found.')


    def save_node(self, name):
        """
        Save the state of a node to a YAML file.

        :param name: The name of the node to save.
        :type name: str
        """
        node_to_save = self.read_node(name)
        if node_to_save is not None:
            state_file = os.path.join(data_dir, name + '.yaml')
            data_to_save = node_to_save.get_data()
            with open(state_file, 'w') as file:
                yaml.dump(data_to_save, file)
            self.get_logger().info('Saved node: ' + name + '.')
        else:
            self.get_logger().info('Node with name ' + name + ' not found.')         


    def load_node(self, name): # TODO: Comprobar que no exista un nodo con ese nombre
        """
        Load the state of a node from a YAML file and create the node.

        :param name: The name of the node to load.
        :type name: str
        """        
        state_file = os.path.join(data_dir, name + '.yaml')
        if os.path.exists(state_file):
            with open(state_file, 'r') as file:
                loaded_data = yaml.load(file, Loader=yaml.FullLoader)
                self.create_node(name, loaded_data)
                self.get_logger().info('Loaded node: ' + str(name))

    def send_command(self, request, response): # TODO: change name to "handle command"
        """
        Handle command requests.

        :param request: The command request.
        :type request: mdb_interfaces.srv.SendCommand_Request
        :param response: The response to the command.
        :type response: mdb_interfaces.srv.SendCommand_Response
        :return: The response to the command.
        :rtype: mdb_interfaces.srv.SendCommand_Response
        """        
        id = int(request.id)
        command = str(request.command)
        name = str(request.name)
        if(id == self.id):
            
            self.get_logger().info('Received request: ' + command)
            
            if(command == 'create'):
                self.create_node(name)
                response.msg = 'Node created: ' + name
            
            elif (command == 'read'):
                read_node = self.read_node(name)
                if read_node is not None:
                    response.msg = 'Node ' + str(name) + ': ' + str(read_node)
                else:
                    response.msg = 'Couldnt read node ' + name + '.'

            elif (command == 'delete'):
                self.delete_node(name)
                response.msg = 'Node deleted: ' + name

            elif (command == 'save'):
                self.save_node(name)
                response.msg = 'Node saved: ' + name

            elif (command == 'load'):
                self.load_node(name)
                response.msg = 'Node loaded: ' + name
            else:
                self.get_logger().info('Wrong command.')
                response.msg = 'Wrong request: ' + command

            return response
        else:
            self.get_logger().info('Request for other executor: ' + command)
            response.msg = 'Request for other executor'
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

    for name, node in execution_node.nodes.items():
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