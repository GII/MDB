import rclpy
from rclpy.node import Node

from core.cognitive_node import CognitiveNode
from cognitive_node_interfaces.srv import SetActivation, SetInputs
from cognitive_node_interfaces.msg import Value

import random

class Perception(CognitiveNode):

    def __init__(self, name='perception', class_name = 'cognitive_nodes.perception.Perception'):

        super().__init__(name, class_name)
        self.register_in_LTM({})

        #N: Value topic
        self.create_publisher(Value, "perception/" + str(name) + "/value", 0) #TODO Implement the message's publication

        # N: Set Activation Service
        self.set_activation_service = self.create_service(
            SetActivation,
            'perception/' + str(name) + '/set_activation',
            self.set_activation_callback
        )

        # N: Set Inputs Service
        self.set_inputs_service = self.create_service(
            SetInputs,
            'perception/' + str(name) + '/set_inputs',
            self.set_inputs_callback
        )

    def calculate_activation(self, perception):
        return random.random()
    
    def set_activation_callback(self, request, response):
        activation = request.activation
        self.get_logger().info('Setting activation ' + str(activation) + '...')
        # TODO: implement logic
        response.set = True
        return response
    
    def set_inputs_callback(self, request, response):
        inputs = request.inputs
        self.get_logger().info('Setting inputs...' + str(inputs) + '...')
        # TODO: implement logic
        response.set = True
        return response

def main(args=None):
    rclpy.init(args=args)

    perception = Perception()

    rclpy.spin(perception)

    perception.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()