import rclpy
from rclpy.node import Node
from mdb.cognitive_node import CognitiveNode
from a_node import ANode
from b_node import BNode
from constants import NODE_TYPES

class LTM:
    def __init__(self):
        super().__init__('LTM')
        self.nodes = {node_type: [] for node_type in NODE_TYPES}
        
        # Register node service
        # self.register_node_service = self.create_service(
        #     RegisterNode,
        #     'register_node',
        #     self.register_node
        # )
        self.last_id = 0
    
    @property
    def a_nodes(self):
        """Return the list of a-nodes."""
        return self.nodes["ANode"]
    
    @property
    def b_nodes(self):
        """Return the list of b-nodes."""
        return self.nodes["BNode"]
    

    def register_node(self, request, response):
        """Add a new node and assign it an ID."""
        self.get_logger().info('Registering node in the LTM')

        # if request.node_type == 'ANode':
        #     new_node = ANode()
        # elif request.node_type == 'BNode':
        #     new_node = BNode()
        # else:
        #     response.success = False
        #     response.message = "Invalid node type: " + request.node_type
        
        # node.assign_id(self.last_id)
        # self.last_id += 1
        # self.nodes[node.type].append(node)
        # return node
        response.id = self.last_id
        self.last_id += 1
        return response

def main():
    pass

if __name__ == '__main__':
    main()
    