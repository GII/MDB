#!/usr/bin/env python
import rospy, sys
from std_msgs.msg import Header, Bool
from moveit_msgs.msg import PlanningScene, AttachedCollisionObject
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from mdb_baxter_policies.srv import PlanMng

class exp_scene():
	def __init__(self):
		self.plan_scene_pb = rospy.Publisher("/planning_scene", PlanningScene, queue_size = 1)
		self.planning_scene = PlanningScene()
		self.attachments = None
		
		self.mod_ps_srver = rospy.Service('/mdb3/baxter/modify_planning_scene', PlanMng, self.handle_mod_ps)

	def handle_mod_ps(self, req):
		if self.manage_element(req.element.data, req.action.data, [req.x.data, req.y.data, req.z.data]):
			return Bool(True)
		else:
			return Bool(False)

	def create_pose(self, pose_data):
		pose = Pose()
		pose.orientation.w = pose_data[0]
		pose.position.x = pose_data[1]
		pose.position.y = pose_data[2]
		pose.position.z = pose_data[3]
		return pose	

	def create_primitive(self, ptype, pdata):
		primitive = SolidPrimitive()
		primitive.type = ptype
		for d in pdata:
			primitive.dimensions.append(d)

		return primitive

	def create_element(self, object_id, pose_data, primitive_data, action):
		self.attachments = AttachedCollisionObject()
		self.attachments.object.header = self.create_header(object_id)
		self.attachments.link_name = "base"
		self.attachments.object.id = object_id

		element_pose = self.create_pose(pose_data)
		element_primitive = self.create_primitive(primitive_data[0], primitive_data[1:])

		self.attachments.object.primitives.append(element_primitive)
		self.attachments.object.primitive_poses.append(element_pose)
		self.attachments.object.operation = action

	def create_header(self, object_id):
		header = Header()
		header.stamp = rospy.Time.now()
		header.frame_id=self.choose_element_frame(object_id)
		return header

	def choose_element_frame(self, arg):
		options = {
			'table':'base',
			'big_obj':'base',
			'small_obj':'base',
			'head_collision':'collision_head_link_1',
		}
		return options[arg]

	def choose_element_pose(self, arg, xyz):
		options = {
			'table':[1.0, 0.61+0.22, 0.0, -0.05+xyz[2]],
			'big_obj':[1.0, xyz[0], xyz[1], -0.05+0.075+0.0095+xyz[2]],
			'small_obj':[1.0, xyz[0], xyz[1], -0.05+0.0325+0.0095+xyz[2]],
			'head_collision':[1.0, 0.0, 0.0, 0.0]
		}
		return options[arg]

	def choose_element_primitive(self, arg):
		options = {
			'table':[SolidPrimitive.BOX, 1.22, 2.442, 0.019],
			'big_obj':[SolidPrimitive.CYLINDER, 0.15, 0.07],
			'small_obj':[SolidPrimitive.CYLINDER, 0.065, 0.03],
			'head_collision':[SolidPrimitive.SPHERE, 0.22],
		}
		return options[arg]

	def choose_element_action(self, arg):
		options = {
			'add':0, 
			'remove':1,
		}
		return options[arg]

	def manage_element(self, element, action, xyz):
		self.create_element(element, self.choose_element_pose(element, xyz), self.choose_element_primitive(element), self.choose_element_action(action))
		self.planning_scene.world.collision_objects.append(self.attachments.object)
		self.planning_scene.is_diff = True
		self.plan_scene_pb.publish(self.planning_scene)
		return True

def exp_scene_server():
	exp_scene()
	rospy.spin()

if __name__ == "__main__":
	rospy.init_node("exp_scene_server")
	exp_scene_server()

