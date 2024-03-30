import rclpy
from rclpy.node import Node

from core.cognitive_node import CognitiveNode
from cognitive_node_interfaces.srv import SetActivation, SetInputs
from cognitive_node_interfaces.msg import Value

import random

class Perception(CognitiveNode):
    """
    Perception class
    """
    def __init__(self, name='perception', class_name = 'cognitive_nodes.perception.Perception', **params):
        """
        Constructor for the Perception class

        Initializes a Perception instance with the given name and registers it in the ltm.
        
        :param name: The name of the Perception instance
        :type name: str¡
        :param class_name: The name of the Perception class
        :type class_name: str
        """
        super().__init__(name, class_name, **params)
        self.register_in_LTM({})

        # We set 1.0 as the default activation value
        self.activation = 1.0

        #N: Value topic
        self.perception_publisher = self.create_publisher(Value, "perception/" + str(name) + "/value", 0) #TODO Implement the message's publication

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

    def calculate_activation(self, perception = None):
        """
        Returns the the activation value of the instance

        :param perception: Perception does not influence the activation of the instance
        :type perception: dict
        :return: The activation of the instance
        :rtype: float
        """
        if self.activation_topic:
            self.publish_activation(self.activation)
        return self.activation
    
    def set_activation_callback(self, request, response):
        """
        Attention mechanisms can modify the activation of a perception instance

        :param request: The request that contains the new activation value
        :type request: cognitive_node_interfaces.srv.SetActivation_Request
        :param response: The response indicating if the activation was set
        :type response: cognitive_node_interfaces.srv.SetActivation_Response
        :return: The response indicating if the activation was set
        :rtype: cognitive_node_interfaces.srv.SetActivation_Response
        """
        activation = request.activation
        self.get_logger().info('Setting activation ' + str(activation) + '...')
        self.activation = activation
        response.set = True
        return response
    
    #TODO Finish the set inputs implementation
    def set_inputs_callback(self, request, response):
        inputs = request.inputs
        self.get_logger().info('Setting inputs...' + str(inputs) + '...')
        # self.destroy_subscription(self.input)
        # TODO: implement the new subscription
        response.set = True
        return response
    
    def read_perception_callback(self, reading): 
        self.get_logger().info("Receiving " + self.name + " = " + str(reading))
        self.reading = reading
        self.process_and_send_reading()

    def process_and_send_reading(self):
        sensor = {}
        value = []
        if isinstance(self.raw.data, list):
            for perception in self.raw.data:
                distance = (
                    perception.distance - self.normalize_values["distance_min"]
                ) / (
                    self.normalize_values["distance_max"]
                    - self.normalize_values["distance_min"]
                )
                angle = (perception.angle - self.normalize_values["angle_min"]) / (
                    self.normalize_values["angle_max"]
                    - self.normalize_values["angle_min"]
                )
                diameter = (
                    perception.diameter - self.normalize_values["diameter_min"]
                ) / (
                    self.normalize_values["diameter_max"]
                    - self.normalize_values["diameter_min"]
                )
                value.append(
                    dict(
                        distance=distance,
                        angle=angle,
                        diameter=diameter,
                        # id=perception.id,
                    )
                )
        else:
            value.append(dict(data=self.raw.data))

        sensor[self.name] = value
        sensor_msg = self.perception_dict_to_msg(sensor)
        self.perception_publisher.publish(sensor_msg)


def main(args=None):
    rclpy.init(args=args)

    perception = Perception()

    rclpy.spin(perception)

    perception.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()