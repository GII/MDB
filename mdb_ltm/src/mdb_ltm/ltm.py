"""
The shiny, all new, MDB 3.0.

Available from (we are still thinking about this...)
Copyright 2017 Richard J. Duro, Jose A. Becerra.
Distributed under the (yes, we are still thinking about this too...).
"""

from __future__ import absolute_import, division, print_function, unicode_literals
from builtins import *  # noqa
import os.path
from copy import copy
from operator import attrgetter
from enum import Enum
import importlib
import threading
from collections import OrderedDict
import yaml
import numpy
import networkx
from matplotlib import pyplot
from matplotlib import cm
import rospy


class NodeColor(Enum):
    """Colors for drawing nodes."""

    Perception = 1.0
    PNode = 3.0
    Goal = 6.0
    ForwardModel = 4.0
    Policy = 10.0
    CNode = 5.0


class NodeColorMap(Enum):
    """Color maps for drawing nodes."""

    Perception = cm.get_cmap("Greys")
    PNode = cm.get_cmap("Purples")
    Goal = cm.get_cmap("Greens")
    ForwardModel = cm.get_cmap("Oranges")
    Policy = cm.get_cmap("Reds")
    CNode = cm.get_cmap("Blues")


class LTM(object):
    """
    The Long Term Memory part of the Brain.

    Attributes
    ----------
        nodes       {type_of_node: [node, ...], ...}
        module_names  {type_of_node: python_module_name, ...}

    """

    def __init__(self):
        """Constructor."""
        self.file_name = None
        self.files = []
        self.nodes = dict(Perception=dict(), PNode=[], CNode=[], Goal=[], ForwardModel=[], Policy=[])
        self.module_names = dict(
            Perception="perception",
            PNode="p_node",
            ForwardModel="forward_model",
            Goal="goal",
            CNode="c_node",
            Policy="policy",
        )
        self.default_class = dict()
        self.default_ros_name_prefix = dict()
        self.there_are_goals = threading.Event()
        self.control_publisher = None
        self.restoring = False
        self.policies_to_test = []
        self.iteration = 0
        self.trial = 0
        self.iterations = None
        self.period = None
        self.trials = None
        self.worlds = None
        self.current_goal = None
        self.current_policy = None
        self.current_world = 0
        self.current_success = 0
        self.graph = networkx.Graph()
        self.graph_node_label = dict()
        self.graph_node_position = dict()
        super().__init__()

    def __getstate__(self):
        """Return the object to be serialize with PyYAML as the result of removing the unpicklable entries."""
        state = self.__dict__.copy()
        del state["files"]
        del state["control_publisher"]
        del state["there_are_goals"]
        return state

    @property
    def perceptions(self):
        """Return the list of perceptions."""
        return self.nodes["Perception"]

    @property
    def p_nodes(self):
        """Return the list of p-nodes."""
        return self.nodes["PNode"]

    @property
    def forward_models(self):
        """Return the list of forward models."""
        return self.nodes["ForwardModel"]

    @property
    def goals(self):
        """Return the list of goals."""
        return self.nodes["Goal"]

    @property
    def c_nodes(self):
        """Return the list of c-nodes."""
        return self.nodes["CNode"]

    @property
    def policies(self):
        """Return the list of policies."""
        return self.nodes["Policy"]

    @classmethod
    def load_memory_dump(cls, file_name):
        """Load a previous LTM memory dump from a file."""
        ltm = yaml.load(open(file_name, "r"), Loader=yaml.CLoader)
        ltm.files = []
        ltm.there_are_goals = threading.Event()
        # Unfortunatly, implementing __setstate__ didn"t work, so I was forced to do this by hand.
        # I think construct_yaml_object is the guilty, because when __setstate__ exists,
        # it calls construct_mapping with deep=True
        for perception in ltm.nodes["Perception"].values():
            perception.init_threading()
            perception.init_ros()
        for node_type, nodes in ltm.nodes.items():
            if node_type != "Perception":
                for node in nodes:
                    if callable(getattr(node, "init_threading", None)):
                        node.init_threading()
                    if callable(getattr(node, "init_ros", None)):
                        node.init_ros()
        ltm.restoring = True
        return ltm

    @classmethod
    def restore(cls, file_name):
        """Return a new LTM object. It loads a LTM memory dump if it exists."""
        if file_name is not None:
            if os.path.isfile(file_name):
                print("Loading a previous LTM memory dump from " + file_name + "...")
                ltm = cls.load_memory_dump(file_name)
            else:
                print(file_name + " does not exist, the name will be used to store a new LTM...")
                ltm = LTM()
        else:
            file_name = "ltm_dump"
            print("Using " + file_name + " to store a new LTM dump...")
            ltm = LTM()
        ltm.file_name = file_name
        return ltm

    @staticmethod
    def __class_from_classname(class_name):
        """Return a class object from a class name."""
        module_string, _, class_string = class_name.rpartition(".")
        node_module = __import__(module_string, fromlist=[bytes(class_string, 'utf-8')])
        # node_module = importlib.import_module('.' + class_string, package=module_string)
        node_class = getattr(node_module, class_string)
        return node_class

    def graph_set_perception_nodes_coordinates(self):
        """Set node position in LTM graph for a Perception node."""
        for idx, same_blood in enumerate(self.nodes["Perception"]):
            self.graph_node_position[same_blood] = [0, idx + 1]

    def graph_set_p_node_nodes_coordinates(self):
        """Set node position in LTM graph for a P-node node."""
        for idx, same_blood in enumerate(self.nodes["PNode"]):
            posx = idx // 3
            posy = idx % 3
            self.graph_node_position[same_blood] = [1 + posx + posy / 3.0, posy]

    def graph_set_goal_nodes_coordinates(self):
        """Set node position in LTM graph for a Goal node."""
        for idx, same_blood in enumerate(self.nodes["Goal"]):
            self.graph_node_position[same_blood] = [3 * (idx + 1), 4]

    def graph_set_forward_model_nodes_coordinates(self):
        """Set node position in LTM graph for a Forward Model node."""
        for idx, same_blood in enumerate(self.nodes["ForwardModel"]):
            self.graph_node_position[same_blood] = [3 * (idx + 1), 6]

    def graph_set_policy_nodes_coordinates(self):
        """Set node position in LTM graph for a Policy node."""
        for idx, same_blood in enumerate(self.nodes["Policy"]):
            self.graph_node_position[same_blood] = [9, idx + 1]

    def graph_set_c_node_nodes_coordinates(self):
        """Set node position in LTM graph for a C-node node."""
        for idx, same_blood in enumerate(self.nodes["CNode"]):
            posx = idx // 3
            posy = idx % 3
            self.graph_node_position[same_blood] = [1 + posx + posy / 3.0, posy + 8]

    def __graph_set_node_properties(self, node):
        """Set node label and position for graphical representation."""
        self.graph_node_label[node] = node.ident
        getattr(self, "graph_set_" + self.module_names[node.type] + "_nodes_coordinates")()

    def __add_node(self, node_type, class_name, ident=None, neighbors=None, **kwargs):
        """Add a new node."""
        # Set the name
        if ident is None:
            ident = node_type + str(len(self.nodes[node_type]))
        # Create the object
        node = self.__class_from_classname(class_name)(ident=ident, node_type=node_type, ltm=self, **kwargs)
        # Add the object to the appropriate list (the perceptions are in a dictionary, not a list)
        if node.type == "Perception":
            self.nodes[node.type][ident] = node
        else:
            self.nodes[node.type].append(node)
        # Add the object to the graph
        self.graph.add_node(node)
        self.__graph_set_node_properties(node)
        # Connect the object in the graph with the right nodes
        if neighbors is None:
            if node.type == "Perception":
                node.neighbors = self.nodes["Goal"] + self.nodes["ForwardModel"] + self.nodes["Policy"]
            else:
                node.neighbors = list(self.nodes["Perception"].values())
        else:
            node.neighbors = neighbors
        for neighbor in node.neighbors:
            neighbor.neighbors.append(node)
            self.graph.add_edge(neighbor, node)
        rospy.loginfo("Created " + node_type + " " + ident)
        return node

    def __add_file(self, file_item):
        if "data" in file_item:
            new_file = self.__class_from_classname(file_item["class"])(
                ident=file_item["id"], file_name=file_item["file"], data=file_item["data"], ltm=self
            )
        else:
            new_file = self.__class_from_classname(file_item["class"])(
                ident=file_item["id"], file_name=file_item["file"], ltm=self
            )
        self.files.append(new_file)

    def __write_headers_in_files(self):
        for file_object in self.files:
            file_object.write_header()

    def __node_exists(self, ident):
        """Check if there is a node with a specific name."""
        for node_type, nodes in self.nodes.items():
            if node_type == "Perception":
                for node in nodes.values():
                    if node.ident == ident:
                        return True
            else:
                for node in nodes:
                    if node.ident == ident:
                        return True
        return False

    def add_node_callback(self, data, node_type):
        """Add a new node without worrying about its class."""
        node_class = None
        if data.command != "new":
            rospy.logerr("Unknown command while processing a message for creating a new node!")
        elif data.execute_service != "" or data.get_service != "":
            node_class = self.__class_from_classname("mdb_ltm." + self.module_names[node_type] + node_type)
        elif data.language != "" and data.language != "python":
            rospy.logerr("Language not supported while processing a message for creating a new node!")
        elif data.class_name != "":
            node_class = self.__class_from_classname(data.class_name)
        else:
            node_class = self.default_class.get(node_type)
            if node_class is None:
                rospy.logerr(
                    "Class name not specified and default value not found while processing a message for "
                    "creating a new node!"
                )
        if node_class is not None:
            if self.__node_exists(data.id):
                rospy.logwarn("Received new %s object, but there is already a node with that name (%s). Ignoring...",
                    node_type, data.id)
            else:
                self.__add_node(
                    node_type=node_type,
                    class_name=node_class,
                    ros_name_prefix=self.default_ros_name_prefix[node_type],
                    ident=data.id,
                    execute_service=data.execute_service,
                    get_service=data.get_service,
                )
                if node_type == "Goal":
                    self.there_are_goals.set()

    def __shutdown(self):
        """Save to disk everything is needed before shutting down."""
        rospy.loginfo("Ending LTM...")
        for file_object in self.files:
            file_object.close()

    def __setup_files(self, files):
        """Load files configuration."""
        for file_item in files:
            self.__add_file(file_item)

    def __setup_topics(self, connectors):
        """Load topic configuration for adding nodes."""
        for connector in connectors:
            self.default_class[connector["data"]] = connector.get("default_class")
            self.default_ros_name_prefix[connector["data"]] = connector["ros_name_prefix"]
            topic = rospy.get_param(connector["ros_name_prefix"] + "_topic")
            message = self.__class_from_classname(rospy.get_param(connector["ros_name_prefix"] + "_msg"))
            callback = getattr(self, connector["callback"])
            callback_args = connector["data"]
            rospy.logdebug("Subscribing to %s...", topic)
            rospy.Subscriber(topic, message, callback=callback, callback_args=callback_args)

    def __setup_nodes(self, nodes):
        """Load initial nodes."""
        for node_type, node_list  in nodes.items():
            rospy.logdebug("Loading %s...", node_type)
            for element in node_list:
                class_name = element["class"]
                ident = element["id"]
                data = element.get("data")
                ros_name_prefix = element.get("ros_name_prefix")
                self.__add_node(
                    node_type=node_type,
                    class_name=class_name,
                    ident=ident,
                    data=data,
                    ros_name_prefix=ros_name_prefix,
                )

    def __setup_control_channel(self, control_channel):
        """Load simulator / robot configuration channel."""
        topic = rospy.get_param(control_channel + "_topic")
        message = self.__class_from_classname(rospy.get_param(control_channel + "_msg"))
        self.control_publisher = rospy.Publisher(topic, message, latch=True, queue_size=0)

    def __setup_experiment(self, experiment):
        """Load experiment configuration."""
        self.iterations = experiment["iterations"]
        self.period = experiment["period"]
        self.trials = experiment["trials"]
        self.worlds = experiment["worlds"]
        self.current_world = self.worlds[0]
        self.__reset_world()

    def __setup(self, log_level, file_name):
        """Init LTM: read ROS parameters, init ROS subscribers and load initial nodes."""
        rospy.init_node("ltm", log_level=getattr(rospy, log_level))
        rospy.loginfo("Starting LTM at iteration " + str(self.iteration) + "...")
        if file_name is None:
            rospy.logerr("No configuration file specified!")
        else:
            if not os.path.isfile(file_name):
                rospy.logerr(file_name + " does not exist!")
            else:
                rospy.loginfo("Loading configuration from %s...", file_name)
                configuration = yaml.load(open(file_name, "r"), Loader=yaml.CLoader)
                self.__setup_files(configuration["LTM"]["Files"])
                self.__setup_topics(configuration["LTM"]["Connectors"])
                if not self.restoring:
                    self.__setup_nodes(configuration["LTM"]["Nodes"])
                if (self.iteration == 0) or self.restoring:
                    self.__write_headers_in_files()
                self.__setup_control_channel(configuration["Control"]["ros_name_prefix"])
                self.__setup_experiment(configuration["Experiment"])
                rospy.on_shutdown(self.__shutdown)

    def __read_perceptions(self):
        """Update the value of every perception."""
        rospy.loginfo("Reading perceptions...")
        sensing = []
        for perception in self.perceptions.values():
            sensing.append(perception.read())
        return sensing

    def __update_activations(self, perception):
        """Update the activation value for all the nodes."""
        rospy.loginfo("Updating activations...")
        for node in self.p_nodes:
            node.update_activation(perception=perception)
        for node in self.forward_models:
            node.update_activation(perception=perception)
        for node in self.goals:
            node.update_activation(perception=perception)
        for node in self.c_nodes:
            node.update_activation(perception=perception)
        for node in self.policies:
            node.update_activation(perception=perception)

    def __random_policy(self):
        """Select a random policy. In order to avoid problems with random numbers" generation, we use a pool."""
        if self.policies_to_test == []:
            self.policies_to_test = copy(self.policies)
        policy = self.policies_to_test[numpy.random.randint(len(self.policies_to_test))]
        return policy

    def __select_policy(self):
        """Select the best policy to be run."""
        policy = max(self.policies, key=attrgetter("activation"))
        if policy.activation == 0:
            policy = self.__random_policy()
        rospy.loginfo("Selecting a policy => " + policy.ident + " (" + str(policy.activation) + ")")
        return policy

    def __select_goal(self):
        """Find the active goal."""
        for goal in self.goals:
            goal.update_success()
        goal = max(self.goals, key=attrgetter("reward"))
        if goal.reward < goal.threshold:
            goal = None
            rospy.loginfo("Successful goal => None")
        else:
            rospy.loginfo("Successful goal => " + goal.ident)
        return goal

    def __add_antipoint(self):
        """Add an anti-point to the p-node that corresponds with the executed policy."""
        cnodes = (node for node in self.current_policy.neighbors if node.type == "CNode" and node.context_is_on())
        for cnode in cnodes:
            pnodes = (
                node for node in cnode.neighbors if node.type == "PNode" and max(node.activation) >= node.threshold
            )
            for pnode in pnodes:
                pnode.add_perception(self.current_policy.perception, -1.0)
                rospy.loginfo("Added anti-point in p-node " + pnode.ident)

    def __add_point(self, sensing):
        """
        Add a point to the p-node that corresponds with the executed policy.

        It is assumed that there are only one goal and one forward model active in the LTM at a given moment of time.
        """
        perception = self.current_policy.perception
        if not perception:
            # If there is not perception in the policy, this means that the P-node was not activated, (see
            # CNode.update_activation()).
            # In this case, if there is some multi-valuated sensor, we log a warning, as this is not solved yet.
            # Policies should be parametizer and, if a policy is randomly executed, it should be called several times,
            # once for each combination of sensor values, so we would know what perception has to be added to the
            # P-node. This would simplify also the simulator and the preprogrammed goals.
            if max([len(i) for i in sensing]) == 1:
                perception = []
                for sensor in sensing:
                    perception.extend(sensor[0])
        if not perception:
            rospy.logwarn("It can't be determined what combination of sensor values caused a goal success, "
                "so it is not possible to add a point / create a new P-node")
        else:
            cnodes = [
                node for node in self.current_policy.neighbors if node.type == "CNode" and node.context_has_reward()]
            if cnodes:
                for cnode in cnodes:
                    pnodes = (node for node in cnode.neighbors if node.type == "PNode")
                    for pnode in pnodes:
                        pnode.add_perception(perception, 1.0)
                        rospy.loginfo("Added point in p-node " + pnode.ident)
            else:
                pnode = self.__add_node("PNode", self.default_class["PNode"])
                pnode.add_perception(perception, 1.0)
                rospy.loginfo("Added point in p-node " + pnode.ident)
                forward_model = max(self.forward_models, key=attrgetter("activation"))
                goal = max(self.goals, key=attrgetter("reward"))
                neighbors = [pnode, forward_model, goal, self.current_policy]
                cnode = self.__add_node("CNode", "mdb_ltm.cnode.CNode", neighbors=neighbors, weight=1.0)
                rospy.logdebug(
                    "New c-node "
                    + cnode.ident
                    + " joining "
                    + pnode.ident
                    + ", "
                    + forward_model.ident
                    + ", "
                    + goal.ident
                    + " and "
                    + self.current_policy.ident
                )

    def __update_policies_to_test(self, drop):
        """Maintenance tasks on the pool of policies used to choose one randomly when needed."""
        if drop:
            if self.current_policy in self.policies_to_test:
                self.policies_to_test.remove(self.current_policy)
        else:
            self.policies_to_test = copy(self.policies)

    def __draw_nodes(self, node_type):
        """Draw nodes of a given type using a specified color map."""
        graph_node_color = []
        nodes = [node for node in self.graph.nodes() if node.type == node_type]
        for node in nodes:
            graph_node_color.append(node.activation)
        circles = networkx.draw_networkx_nodes(
            self.graph,
            self.graph_node_position,
            nodelist=nodes,
            node_size=1200,
            node_color=graph_node_color,
            alpha=0.5,
            cmap=NodeColorMap[node_type].value,
            vmin=0.0,
            vmax=1.0,
            label=node_type,
        )
        if circles:
            circles.set_edgecolor("k")

    @staticmethod
    def __graph_set_edge_properties(edges):
        """Set edge width and color for graphical representation."""
        graph_edge_width = []
        graph_edge_color = []
        for edge in edges:
            if edge[0].type == "CNode" or edge[1].type == "CNode":
                graph_edge_width.append(1)
                if edge[0].type == "PNode" or edge[1].type == "PNode":
                    graph_edge_color.append("m")
                elif edge[0].type == "Goal" or edge[1].type == "Goal":
                    graph_edge_color.append("g")
                elif edge[0].type == "ForwardModel" or edge[1].type == "ForwardModel":
                    graph_edge_color.append("b")
                elif edge[0].type == "Policy" or edge[1].type == "Policy":
                    graph_edge_color.append("r")
            else:
                graph_edge_width.append(0.2)
                graph_edge_color.append("k")
        return graph_edge_width, graph_edge_color

    def __show(self):
        """Plot the LTM structure."""
        pyplot.get_current_fig_manager().window.geometry("800x600-0-0")
        pyplot.clf()
        pyplot.ioff()
        pyplot.axis("off")
        pyplot.subplots_adjust(left=0.0, right=1.0, bottom=0.0, top=0.9)
        for node_type in self.nodes:
            if node_type != "Goal":
                self.__draw_nodes(node_type)
        graph_edge_width, graph_edge_color = self.__graph_set_edge_properties(self.graph.edges())
        networkx.draw_networkx_edges(
            self.graph, self.graph_node_position, width=graph_edge_width, edge_color=graph_edge_color, alpha=0.5
        )
        networkx.draw_networkx_labels(self.graph, self.graph_node_position, labels=self.graph_node_label, font_size=8)
        goal = max(self.goals, key=attrgetter("reward"))
        if goal.reward == 0:
            pyplot.title(
                "GOAL: None WORLD: "
                + self.current_world
                + "\nITERATION: "
                + str(self.iteration)
                + " REWARD: "
                + str(self.current_success)
                + "\nPOLICY: "
                + self.current_policy.ident,
                fontsize=10,
            )
        else:
            pyplot.title(
                "GOAL: "
                + goal.ident
                + " WORLD: "
                + self.current_world
                + "\nITERATION: "
                + str(self.iteration)
                + " REWARD: "
                + str(self.current_success)
                + "\nPOLICY: "
                + self.current_policy.ident,
                fontsize=10,
            )
        # Legend is disable due to all nodes have the same color due to, when we paint nodes, we change color map.
        # pyplot.legend(
        #     loc="bottom right", shadow=True, fancybox=True, fontsize=10, labelspacing=3, bbox_to_anchor=(1.30, 0.5))
        # You will not get any graph on the screen without this line
        pyplot.pause(0.0001)

    def sensorial_changes(self):
        """Return false if all perceptions have the same value as the previous step. True otherwise."""
        for sensor in self.perceptions.values():
            for perception, perception_old in zip(sensor.value, sensor.old_value):
                if isinstance(perception, dict):
                    for attribute in perception:
                        difference = abs(perception[attribute] - perception_old[attribute])
                        if difference >= 0.01:
                            return True
                else:
                    if abs(perception[0] - perception_old[0]) >= 0.01:
                        return True
        return False

    def __statistics(self):
        """Update some statistics and write them to files."""
        for file_object in self.files:
            file_object.write()

    def __reset_world(self):
        """Reset the world if necessary, according to the experiment parameters."""
        changed = False
        self.trial += 1
        if self.trial == self.trials or self.current_success >= 0.9:
            self.trial = 0
            changed = True
        if ((self.iteration % self.period) == 0) or self.restoring:
            self.current_world = self.worlds[(self.iteration // self.period) % len(self.worlds)]
            self.trial = 0
            self.restoring = False
            changed = True
        if changed:
            self.control_publisher.publish(world=self.current_world, reward=(self.current_success >= 0.9))
        return changed

    def run(self, seed=None, log_level="INFO", plot=False):
        """Start the LTM part of the brain."""
        try:
            self.__setup(log_level, seed)
            rospy.loginfo("Running LTM...")
            sensing = self.__read_perceptions()
            while (not rospy.is_shutdown()) and (self.iteration <= self.iterations):
                rospy.loginfo("*** ITERATION: " + str(self.iteration) + " ***")
                if not self.goals:
                    self.there_are_goals.wait()
                self.__update_activations(sensing)
                self.current_policy = self.__select_policy()
                self.current_policy.execute()
                sensing = self.__read_perceptions()
                self.current_goal = self.__select_goal()
                if self.current_goal is not None:
                    drop_policy = False
                    self.current_success = self.current_goal.reward
                    self.__add_point(sensing)
                else:
                    drop_policy = True
                    self.current_success = 0.0
                    if self.current_policy.activation >= self.current_policy.threshold:
                        self.__add_antipoint()
                self.__update_policies_to_test(drop=drop_policy)
                if plot:
                    self.__show()
                self.__statistics()
                self.iteration += 1
                if self.__reset_world():
                    sensing = self.__read_perceptions()
        except rospy.ROSInterruptException:
            rospy.logerr("Exception caught! Or you pressed CTRL+C or something went wrong...")
