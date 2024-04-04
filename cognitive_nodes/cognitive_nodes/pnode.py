import rclpy
from core.cognitive_node import CognitiveNode
from core.utils import class_from_classname, perception_msg_to_dict, separate_perceptions
from cognitive_node_interfaces.srv import AddPoint

class PNode(CognitiveNode):
    """
    PNode class
    """
    def __init__(self, name= 'pnode', class_name = 'cognitive_nodes.pnode.PNode', space_class = None, space = None, **params):
        """
        Constructor for the PNode class.
        
        Initializes a PNode with the given name and registers it in the LTM.
        It also creates a service for adding points to the node.
        
        :param name: The name of the PNode.
        :type name: str
        :param class_name: The name of the PNode class.
        :type class_name: str
        :param space_class: The class of the space used to define the PNode
        :type space_class: str
        :param space: The space used to define the PNode
        :type space: cognitive_nodes.space
        """
        super().__init__(name, class_name, **params)
        self.spaces = [space if space else class_from_classname(space_class)(name + " space")]
        self.register_in_LTM({})
        self.add_point_service = self.create_service(AddPoint, 'pnode/' + str(name) + '/add_point', self.add_point_callback)


    def add_point_callback(self, request, response):
        """
        Callback method for adding a point (or anti-point) to a specific PNode.

        :param request: The request that contains the point that is added and its confidence.
        :type request: cognitive_node_interfaces.srv.AddPoint_Request
        :param response: The response indicating if the point was added to the PNode.
        :type respone: core_interfaces.srv.AddPoint_Response
        :return: The response indicating if the point was added to the PNode.
        :rtype: cognitive_node_interfaces.srv.AddPoint_Response
        """
        point_msg = request.point
        confidence = request.confidence
        point = perception_msg_to_dict(point_msg)
        self.add_point(point,confidence)
        self.get_logger().info('Adding point: ' + str(point) + 'Confidence: ' + str(confidence))
        response.added = True

        return response
    
    def add_point(self, point, confidence):
        """
        Add a new point (or anti-point) to the PNode.
        
        :param point: The point that is added to the PNode
        :type point: dict
        :param confidence: Indicates if the perception added is a point or an antipoint.
        :type confidence: float
        """
        space = self.get_space(point)
        if not space:
            space = self.spaces[0].__class__()
            self.spaces.append(space)
        added_point_pos = space.add_point(point, confidence)
        
    def calculate_activation(self, perception=None):
        """
        Calculate the new activation value for a given perception

        :param perception: The perception for which PNode activation is calculated.
        :type perception: dict
        :return: If there is space, returns the activation of the PNode. If not, returns 0
        :rtype: float
        """
        if perception:
            activations = []
            perceptions = separate_perceptions(perception)
            for perception_line in perceptions:
                space = self.get_space(perception)
                if space:
                    activation_value = space.get_probability(perception_line)
                else:
                    activation_value = 0.0

                activations.append(activation_value) 
            
            self.activation == activations[0] if len(activations) == 1 else activations
        
        if self.activation_topic:
            self.publish_activation(self.activation)
            
        return self.activation

    def get_space(self, perception):
        """
        Return the compatible space with perception.
        (Ugly hack just to see if this works. In that case, everything need to be checked to reduce the number of
        conversions between sensing, perception and space.)

        :param perception: The perception for which PNode activation is calculated.
        :type perception: dict
        :return: If there is space, returns it. If not, returns None.
        :rtype: cognitive_nodes.space or None
        """
        temp_space = self.spaces[0].__class__()
        temp_space.add_point(perception, 1.0)
        for space in self.spaces:
            if (not space.size) or space.same_sensors(temp_space):
                return space
        return None

def main(args = None):
    rclpy.init(args=args)

    pnode = PNode()

    rclpy.spin(pnode)

    pnode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()