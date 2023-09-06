import rclpy
from rclpy.node import Node
from mdb.cognitive_node import CognitiveNode

class LTM:
    def __init__(self):
        super().__init__('LTM')
        self.a_nodes = {}


    def register_node(self, node_type, node):
        name = node.get_name()
        if node_type == 'a_node':
            self.a_nodes[name] = node
    