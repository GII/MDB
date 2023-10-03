import rclpy
from mdb.cognitive_node import CognitiveNode
from mdb.utils import class_from_classname
from mdb_interfaces.srv import AddPerception

class PNode(CognitiveNode):
    """
    A subspace of the input space (sensorial or result of a redescription process).

    This subspace is linked to every node for which it is relevant,
    activating them when a new perception pertaining to this subspace occurs.
    This class is migrating to be able to aggregate different input spaces.
    """

    def __init__(self, space_class=None, space=None, **kwargs):
        """Initialize."""
        self.spaces = [space if space else class_from_classname(space_class)(ident=kwargs.get("ident") + " space")]
        super().__init__(**kwargs)

    def __init__(self, name='pnode'):
        super().__init__(name, 'mdb.pnode.PNode')
        self.register_in_LTM([],[]) #¿Qué era subscribing y publishing?
        self.add_perception_service = self.create_service(AddPerception, 'pnode/' + str(name) + '/add_perception', self.add_perception_callback)


    # def publish(self, message=None, first_time=False):
    #     """Publish node information."""
    #     message = self.node_message()
    #     space = self.spaces[0]
    #     if not isinstance(space.members, list):
    #         message.names = space.members.dtype.names
    #     else:
    #         message.names = []
    #     if first_time and not isinstance(message.names, list):
    #         point_message = self.data_message()
    #         point_message.command = "new"
    #         point_message.id = self.ident
    #         point_array = structured_to_unstructured(space.members)
    #         confidence_array = space.memberships
    #         for point, confidence in zip(point_array, confidence_array):
    #             point_message.point = point
    #             point_message.confidence = confidence
    #             self.data_publisher.publish(point_message)
    #     super().publish(message, first_time)
    
    def add_perception_callback(self, request, response):
        perception = request.perception
        confidence = request.confidence
        self.add_perception(perception,confidence)

        self.get_logger().info('Adding perception: ' + perception)

        response.added = True

        return response
    
    def add_perception(self, perception, confidence):
        """Add a new point to the p-node."""
        space = self.get_space(perception)
        if not space:
            space = self.spaces[0].__class__()
            self.spaces.append(space)
        added_point_pos = space.add_point(perception, confidence)
        point_message = self.data_message()
        point_message.id = self.ident
        if added_point_pos != -1:
            point_message.command = "new"
            point_message.point = self.spaces[0].members[added_point_pos]
            point_message.confidence = self.spaces[0].memberships[added_point_pos]
            self.data_publisher.publish(point_message)

    def calculate_activation(self, perception=None):
        """Calculate the new activation value."""
        space = self.get_space(perception)
        if space:
            return space.get_probability(perception)
        return 0

    def get_space(self, perception):
        """Return the compatible space with perception."""
        # Ugly hack just to see if this works. In that case, everything need to be checked to reduce the number of
        # conversions between sensing, perception and space.
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