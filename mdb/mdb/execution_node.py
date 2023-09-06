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

config_dir = '/home/cristina/ros2_ws/src/mdb/config'

class ExecutionNode(Node):

    def __init__(self, executor, id):

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

        # # Send command publisher (cuando muevo un nodo, lo borro aquí y envío la orden de crearlo en otro lado)
        # self.send_command_publisher = self.create_publisher(
        #     Command,
        #     'commands',
        #     10
        # )

    def create_node(self, name, data=None): # TODO: diferenciar tipos de nodo
        if data == None:
            
            new_node = ANode(name)
        else:
            new_node = ANode(name, **data)

        self.nodes[name] = new_node
        self.executor.add_node(new_node)
        self.get_logger().info('Added node: ' + str(new_node.get_name()))

    def read_node(self, name):
        if name in self.nodes:
            return self.nodes.get(name)
        else:
            return None

    # def update_node(self, name, state):
    #     node_to_update = self.read_node(name)
    #     if node_to_update is not None:
    #         self.get_logger().info("new state: " + str(state))
    #         self.get_logger().info("current state: " + str(node_to_update.get_state()))
    #         node_to_update.set_state(state)
    #         self.get_logger().info('Updated node: ' + name + '.')
    #     else:
    #         self.get_logger().info('Node with name ' + name + ' not found.')
    #     return node_to_update

    def delete_node(self, name):
        if name in self.nodes:
            node_to_delete = self.nodes.pop(name)
            self.executor.remove_node(node_to_delete)
            node_to_delete.destroy_node()
            self.get_logger().info('Deleted node: ' + name + '.')
        else:
            self.get_logger().info('Node with name ' + name + ' not found.')


    def save_node(self, name):
        node_to_save = self.read_node(name)
        if node_to_save is not None:
            state_file = os.path.join(config_dir, name + '.yaml')
            data_to_save = node_to_save.get_data()
            with open(state_file, 'w') as file:
                yaml.dump(data_to_save, file)
            self.get_logger().info('Saved node: ' + name + '.')
        else:
            self.get_logger().info('Node with name ' + name + ' not found.')         


    def load_node(self, name): # TODO: Comprobar que no exista un nodo con ese nombre
        state_file = os.path.join(config_dir, name + '.yaml')
        if os.path.exists(state_file):
            with open(state_file, 'r') as file:
                loaded_data = yaml.load(file, Loader=yaml.FullLoader)
                self.create_node(name, loaded_data)
                self.get_logger().info('Loaded node: ' + str(name))


    # Send command service
    def send_command(self, request, response):
        id = int(request.id)
        command = str(request.command)
        name = str(request.name)
        if(id == self.id):
            self.get_logger().info('Received request: ' + command)
            response.msg = 'Executed request: ' + command
            if(command == 'create'):
                self.create_node(name)
                response.msg = 'Node created: ' + name
            elif (command == 'read'):
                read_node = self.read_node(name)
                if read_node is not None:
                    response.msg = 'Node ' + str(name) + ': ' + str(read_node)
                else:
                    response.msg = 'Couldnt read node ' + name + '.'
            elif (command == 'update'):
                # state = yaml.safe_load(request.state)
                # updated_node = self.update_node(name, state)
                # response.msg = 'Node updated. New state: ' + str(updated_node.get_state())
                pass # TODO implementar update
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
    executor = SingleThreadedExecutor()
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