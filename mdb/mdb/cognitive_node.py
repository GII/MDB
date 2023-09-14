import yaml
from rclpy.node import Node

class CognitiveNode(Node):
    """
    A base class for cognitive nodes in the system.

    This class extends the `rclpy.node.Node` class and provides some
    common functionality for cognitive nodes.

    :param name: The name of the node.
    """

    def __init__(self, name):   # TODO: add state and type
        """
        Initialize a CognitiveNode.

        :param name: The name of the node.
        """
        super().__init__(name)

    def assign_id(self, id):
        """
        Assign an ID to the node.

        :param id: The ID (an integer) to be assigned.
        :type id: int
        """
        self.id = id

    def get_data(self):
        """
        Get the data associated with the node.

        This method returns a dictionary containing the attributes of
        the node, excluding private attributes and the 'subscription'
        attribute found in the ANode class.

        :return: A dictionary with node data.
        :rtype: dict
        """
        node_data = self.__dict__.copy()
        keys_to_delete = [key for key in node_data.keys() if key.startswith('_')]
        for key in keys_to_delete:
            del node_data[key]
        del node_data['subscription']
        return node_data

    def __str__(self):
        """
        Returns a YAML representation of the node's data.

        :return: YAML representation of the node's data.
        :rtype: str
        """
        data = self.get_data()
        return yaml.dump(data, default_flow_style=False)

def main(args=None):
    pass

if __name__ == '__main__':
    main()
