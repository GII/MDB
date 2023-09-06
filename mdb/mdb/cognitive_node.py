import json
from rclpy.node import Node

class CognitiveNode(Node):

    def __init__(self, name):   # TODO: add state and type
        super().__init__(name)

    def get_data(self):
        node_data = self.__dict__.copy()
        keys_to_delete = [key for key in node_data.keys() if key.startswith('_')]
        for key in keys_to_delete:
            del node_data[key]
        del node_data['subscription']
        return node_data

    def update_state(new_state):
        pass

    def __str__(self):
        data = self.get_data()
        return json.dumps(data)

def main(args=None):
    pass

if __name__ == '__main__':
    main()
