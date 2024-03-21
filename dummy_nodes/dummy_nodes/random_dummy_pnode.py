from cognitive_nodes.pnode import PNode
import random

class RandomDummyPNode(PNode):
    def __init__(self, name='pnode', class_name='cognitive_nodes.pnode.PNode', space_class=None, space=None, **params):
        super().__init__(name, class_name, space_class, space, **params)

    def calculate_activation(self, perception=None):
        self.activation = random.random()
        if self.activation_topic:
            self.publish_activation(self.activation)
        return self.activation