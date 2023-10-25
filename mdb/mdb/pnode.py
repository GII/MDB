import rclpy
from mdb.cognitive_node import CognitiveNode
from mdb.utils import class_from_classname
from cognitive_node_interfaces.srv import AddPoint
from cognitive_node_interfaces.srv import SetActivationTopic

class PNode(CognitiveNode):
    """
    PNode class
    """

    def __init__(self, name='pnode', class_name = 'mdb.pnode.PNode', space_class = None, space = None):
        """
        Constructor for the PNode class.
        
        Initializes a PNode with the given name and registers it in the LTM.
        It also creates a service for adding points to the node.
        
        :param name: The name of the PNode.
        :type name: str
        :param class_name: The name of the PNode class.
        :type class_name: str
        :param space_class: The class of the space used to define the PNode
        :type space_class: str or None
        :param space: The space used to define the PNode
        :type space: mdb.space or None
        """
        super().__init__(name, class_name)
        self.spaces = [space if space else class_from_classname(space_class)(name + " space")]
        self.register_in_LTM([],[])
        self.add_point_service = self.create_service(AddPoint, 'pnode/' + str(name) + '/add_point', self.add_point_callback)
        self.set_activation_topic_service= self.create_service(SetActivationTopic, 'pnode/' + str(name) + '/set_activation_topic', self.set_activation_topic_callback)


    def add_point_callback(self, request, response): # TODO: Consider error adding point
        """
        Callback function for adding a point (or anti-point) to a specific PNode.

        :param request: The request that contains the point that is added and its confidence.
        :type request: mdb_interfaces.srv.AddPoint_Request
        :param response: The response indicating if the point was added to the PNode.
        :type respone: mdb_interfaces.srv.AddPoint_Response
        :return: The response indicating if the point was added to the PNode.
        :rtype: mdb_interfaces.srv.AddPoint_Response
        """
        point = request.point
        confidence = request.confidence
        self.add_point(point,confidence)

        self.get_logger().info('Adding point: ' + point + 'Confidence: ' + confidence)

        response.added = True

        return response
    
    def add_point(self, point, confidence):
        """
        Add a new point (or antipoint) to the PNode.
        
        :param point: The point that is added to the PNode
        :type point: Any
        :param confidence: Indicates if the perception added is a point or an antipoint.
        :type confidence: float
        """
        space = self.get_space(point)
        if not space:
            space = self.spaces[0].__class__()
            self.spaces.append(space)
        added_point_pos = space.add_point(point, confidence)
        point_message = self.data_message()
        point_message.id = self.ident
        if added_point_pos != -1:
            point_message.command = "new"
            point_message.point = self.spaces[0].members[added_point_pos]
            point_message.confidence = self.spaces[0].memberships[added_point_pos]
            self.data_publisher.publish(point_message)

    def calculate_activation(self, perception=None):
        """
        Calculate the new activation value.

        :param perception: The perception for which PNode activation is calculated.
        :type perception: Any or None
        :return: If there is space, returns the activation of the PNode. If not, returns 0
        :rtype: float
        """
        space = self.get_space(perception)
        if space:
            return space.get_probability(perception)
        return 0

    def get_space(self, perception):
        """
        Return the compatible space with perception.
        (Ugly hack just to see if this works. In that case, everything need to be checked to reduce the number of
        conversions between sensing, perception and space.)

        :param perception: The perception for which PNode activation is calculated.
        :type perception: Any
        :return: If there is space, returns it. If not, returns None.
        :rtype: Any or None
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