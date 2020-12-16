"""
MDB.

https://github.com/GII/MDB
"""

# Python 2 compatibility imports
from __future__ import absolute_import, division, print_function, unicode_literals
from future import standard_library
from future.utils import bytes_to_native_str

standard_library.install_aliases()
from builtins import *  # noqa pylint: disable=unused-wildcard-import,wildcard-import

# Standard imports
import os.path
import random
import sys
from copy import copy
from operator import attrgetter
import threading
from collections import OrderedDict

# Library imports
import yaml
import yamlloader
import numpy
import rospy


class LTM(object):
    """
    The Long Term Memory part of the Brain.

    Attributes:
    ----------
        nodes       {type_of_node: [node, ...], ...}
        module_names  {type_of_node: python_module_name, ...}

    """

    def __init__(self):
        """Init attributes when a new object is created."""
        self.file_name = None
        self.files = []
        self.nodes = OrderedDict(Perception=OrderedDict(), PNode=[], CNode=[], Goal=[], ForwardModel=[], Policy=[])
        self.module_names = OrderedDict(
            Perception="perception",
            PNode="p_node",
            ForwardModel="forward_model",
            Goal="goal",
            CNode="c_node",
            Policy="policy",
        )
        # Init thread syncronizing stuff
        self.reward_semaphore = None
        self.reward_event = None
        self.init_threading()
        #
        self.default_class = OrderedDict()
        self.default_ros_node_prefix = OrderedDict()
        self.default_ros_data_prefix = OrderedDict()
        self.control_publisher = None
        self.info_publisher = None
        self.restoring = False
        self.policies_to_test = []
        self.iteration = 0
        self.trial = 0
        self.iterations = None
        self.period = None
        self.trials = None
        self.worlds = None
        self.current_world = 0
        self.current_policy = None
        self.received_reward = 0
        self.current_reward = 0
        self.current_goal = None
        self.paused = False
        super().__init__()

    def __getstate__(self):
        """Return the object to be serialize with PyYAML as the result of removing the unpicklable entries."""
        state = self.__dict__.copy()
        del state["files"]
        del state["control_publisher"]
        del state["reward_semaphore"]
        del state["reward_event"]
        return state

    def init_threading(self):
        """Create needed stuff to synchronize threads."""
        self.reward_semaphore = threading.Semaphore()
        self.reward_event = threading.Event()

    @property
    def perceptions(self):
        """Return the dictionary of perceptions."""
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

    @staticmethod
    def load_memory_dump(file_name):
        """Load a previous LTM memory dump from a file."""
        ltm = yaml.load(open(file_name, "r", encoding="utf-8"), Loader=yamlloader.ordereddict.CLoader)
        ltm.files = []
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
    def class_from_classname(class_name):
        """Return a class object from a class name."""
        module_string, _, class_string = class_name.rpartition(".")
        node_module = __import__(module_string, fromlist=[bytes_to_native_str(bytes(class_string, "utf-8"))])
        # node_module = importlib.import_module('.' + class_string, package=module_string)
        node_class = getattr(node_module, class_string)
        return node_class

    def add_node(self, node_type=None, class_name=None, ident=None, neighbors=None, **kwargs):
        """Add a new node."""
        # Set the name
        if ident is None:
            ident = node_type + str(len(self.nodes[node_type]))
        # Create the object
        node = self.class_from_classname(class_name)(ident=ident, node_type=node_type, ltm=self, **kwargs)
        # Add the object to the appropriate list (the perceptions are in a dictionary, not a list)
        if node.type == "Perception":
            self.nodes[node.type][ident] = node
        else:
            self.nodes[node.type].append(node)
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
        rospy.loginfo("Created " + node_type + " " + ident)
        return node

    def add_file(self, file_item):
        """Process a file entry (create the corresponding object) in the configuration."""
        if "data" in file_item:
            new_file = self.class_from_classname(file_item["class"])(
                ident=file_item["id"], file_name=file_item["file"], data=file_item["data"], ltm=self
            )
        else:
            new_file = self.class_from_classname(file_item["class"])(
                ident=file_item["id"], file_name=file_item["file"], ltm=self
            )
        self.files.append(new_file)

    def write_headers_in_files(self):
        """Write the header of each open file."""
        for file_object in self.files:
            file_object.write_header()

    def node_exists(self, ident):
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
        if data._connection_header["callerid"] != rospy.get_name() and data.command == "new":
            node_class = None
            if data.execute_service != "" or data.get_service != "":
                node_class = self.class_from_classname("mdb_ltm." + self.module_names[node_type] + node_type)
            elif data.language != "" and data.language != "python":
                rospy.logerr("Language not supported while processing a message for creating a new node!")
            elif data.class_name != "":
                node_class = self.class_from_classname(data.class_name)
            else:
                node_class = self.default_class.get(node_type)
                if node_class is None:
                    rospy.logerr(
                        "Class name not specified and default value not found while processing a message for "
                        "creating a new node!"
                    )
            if node_class is not None:
                if self.node_exists(data.id):
                    rospy.logwarn(
                        "Received new %s object, but there is already a node with that name (%s). Ignoring...",
                        node_type,
                        data.id,
                    )
                else:
                    self.add_node(
                        node_type=node_type,
                        class_name=node_class,
                        ros_node_prefix=self.default_ros_node_prefix[node_type],
                        ros_data_prefix=self.default_ros_data_prefix[node_type],
                        ident=data.id,
                        execute_service=data.execute_service,
                        get_service=data.get_service,
                    )

    def shutdown(self):
        """Save to disk everything is needed before shutting down."""
        rospy.loginfo("Ending LTM...")
        for file_object in self.files:
            file_object.close()

    def setup_files(self, files):
        """Load files configuration."""
        for file_item in files:
            self.add_file(file_item)

    def setup_topics(self, connectors):
        """Load topic configuration for adding nodes."""
        for connector in connectors:
            self.default_class[connector["data"]] = connector.get("default_class")
            self.default_ros_node_prefix[connector["data"]] = connector.get("ros_node_prefix")
            self.default_ros_data_prefix[connector["data"]] = connector.get("ros_data_prefix")
            if self.default_ros_node_prefix[connector["data"]] is not None:
                topic = rospy.get_param(connector["ros_node_prefix"] + "_topic")
                message = self.class_from_classname(rospy.get_param(connector["ros_node_prefix"] + "_msg"))
                callback = getattr(self, connector["callback"])
                callback_args = connector["data"]
                rospy.logdebug("Subscribing to %s...", topic)
                rospy.Subscriber(topic, message, callback=callback, callback_args=callback_args)

    def setup_nodes(self, nodes):
        """Load initial nodes."""
        for node_type, node_list in nodes.items():
            rospy.logdebug("Loading %s...", node_type)
            for element in node_list:
                class_name = element["class"]
                ident = element["id"]
                data = element.get("data")
                ros_data_prefix = element.get("ros_data_prefix")
                if not ros_data_prefix:
                    ros_data_prefix = self.default_ros_data_prefix[node_type]
                self.add_node(
                    node_type=node_type,
                    class_name=class_name,
                    ros_node_prefix=self.default_ros_node_prefix[node_type],
                    ident=ident,
                    data=data,
                    ros_data_prefix=ros_data_prefix,
                )

    def control_callback(self, message):
        """Read a command published in the control topic and find out if something must be done."""
        if message.command == "publish_ltm":
            for nodes in self.nodes.values():
                if isinstance(nodes, OrderedDict):
                    for node in nodes.values():
                        node.publish(first_time=True)
                elif isinstance(nodes, list):
                    for node in nodes:
                        node.publish(first_time=True)
        elif message.command == "pause":
            self.paused = True
        elif message.command == "continue":
            self.paused = False

    def reward_callback(self, reward):
        """Read a reward from the outer world."""
        self.reward_semaphore.acquire()
        rospy.logdebug("Receiving reward = " + str(reward.data))
        self.received_reward = reward.data
        self.reward_event.set()
        self.reward_semaphore.release()

    def setup_control_channel(self, control_channel):
        """Load simulator / robot configuration channel."""
        topic = rospy.get_param(control_channel["control_prefix"] + "_topic")
        message = self.class_from_classname(rospy.get_param(control_channel["control_prefix"] + "_msg"))
        self.control_publisher = rospy.Publisher(topic, message, latch=True, queue_size=0)
        rospy.Subscriber(topic, message, callback=self.control_callback)
        topic = rospy.get_param(control_channel["info_prefix"] + "_topic")
        message = self.class_from_classname(rospy.get_param(control_channel["info_prefix"] + "_msg"))
        self.info_publisher = rospy.Publisher(topic, message, latch=True, queue_size=0)
        topic = rospy.get_param(control_channel["reward_prefix"] + "_topic")
        message = self.class_from_classname(rospy.get_param(control_channel["reward_prefix"] + "_msg"))
        rospy.logdebug("Subscribing to %s...", topic)
        rospy.Subscriber(topic, message, callback=self.reward_callback)

    def setup_experiment(self, experiment):
        """Load experiment configuration."""
        self.iterations = experiment["iterations"]
        self.period = experiment["period"]
        self.trials = experiment["trials"]
        self.worlds = experiment["worlds"]
        self.current_world = self.worlds[0]
        self.reset_world()

    def setup(self, log_level, file_name):
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
                configuration = yaml.load(open(file_name, "r", encoding="utf-8"), Loader=yamlloader.ordereddict.CLoader)
                self.setup_files(configuration["LTM"]["Files"])
                self.setup_topics(configuration["LTM"]["Connectors"])
                if not self.restoring:
                    self.setup_nodes(configuration["LTM"]["Nodes"])
                if (self.iteration == 0) or self.restoring:
                    self.write_headers_in_files()
                self.setup_control_channel(configuration["Control"])
                self.setup_experiment(configuration["Experiment"])
                rospy.on_shutdown(self.shutdown)

    def read_perceptions(self):
        """Update the value of every perception."""
        rospy.loginfo("Reading perceptions...")
        sensing = OrderedDict()
        for perception in self.perceptions.values():
            sensing[perception.ident] = perception.read()
        return sensing

    def process_value_functions(self):
        """Execute value functions of active goals, which will produce sub-goals activation."""
        active_goals = set(goal for goal in self.goals if goal.max_activation > 0.0)
        while active_goals:
            active_goals.union(active_goals.pop().exec_value_functions())

    def update_activations(self, perception, new_sensings=True):
        """Update the activation value for all the nodes."""
        rospy.loginfo("Updating activations...")
        if new_sensings:
            for p_node in self.p_nodes:
                p_node.update_activation(perception=perception)
            for forward_model in self.forward_models:
                forward_model.update_activation(perception=perception)
        for goal in self.goals:
            goal.update_activation(perception=perception)
        self.process_value_functions()
        for c_node in self.c_nodes:
            c_node.update_activation(perception=perception)
        for policy in self.policies:
            policy.update_activation(perception=perception)

    def do_forward_prospection(self, pnodes_to_consider, remaining_pnodes, path_goals, path_policies, search_new=False):
        """
        Perform forward prospection starting with currently activated P-nodes and trying to find a useful policy.

        This methods explotes information regarding what goals are content inside a P-node.

        Notes:
        -----
        - Currently, prospection is carried out only for paths with active forward models. Otherwise,
        weird things can happen, because goals can be inside P-nodes due to pure chance.
        - A goal can have several P-nodes connected to it through different C-nodes, so it could appear more than
        once in the path. We don't allow this to avoid loops.

        """
        new_knowledge = False
        for p_node in pnodes_to_consider:
            c_nodes = [node for node in p_node.neighbors if node.type == "CNode"]
            for c_node in c_nodes:
                forward_model = c_node.forward_model
                goal = c_node.goal
                policy = c_node.policy
                if (forward_model.max_activation > forward_model.threshold) and (goal not in path_goals):
                    new_path_goals = path_goals + [goal]
                    new_path_policies = path_policies + [policy]
                    if goal.max_activation > goal.threshold:
                        goal.add_value_function(new_path_goals)
                        new_knowledge = True
                        value_function_text = "New value function"
                        for goal, policy in zip(new_path_goals, new_path_policies):
                            value_function_text += " => (" + policy.ident + ") " + goal.ident
                        rospy.loginfo(value_function_text)
                    if search_new:
                        goal.update_embedded(self.p_nodes)
                    new_pnodes_to_consider = remaining_pnodes.intersection(goal.embedded)
                    new_remaining_pnodes = remaining_pnodes.difference(goal.embedded)
                    self.do_forward_prospection(
                        pnodes_to_consider=new_pnodes_to_consider,
                        remaining_pnodes=new_remaining_pnodes,
                        path_goals=new_path_goals,
                        path_policies=new_path_policies,
                        search_new=search_new,
                    )
        return new_knowledge

    def do_prospection(self):
        """Perform prospection trying to find a useful policy."""
        pnodes_to_consider = {p_node for p_node in self.p_nodes if max(p_node.activation) > 0.0}
        remaining_pnodes = {p_node for p_node in self.p_nodes if max(p_node.activation) <= 0.0}
        new_knowledge = self.do_forward_prospection(
            pnodes_to_consider=pnodes_to_consider,
            remaining_pnodes=remaining_pnodes,
            path_goals=[],
            path_policies=[],
            search_new=False,
        )
        if not new_knowledge:
            new_knowledge = self.do_forward_prospection(
                pnodes_to_consider=pnodes_to_consider,
                remaining_pnodes=remaining_pnodes,
                path_goals=[],
                path_policies=[],
                search_new=True,
            )
        return new_knowledge

    def get_feasible_goals(self):
        """
        Find out goals that can be satisfied (or their first subgoal can be satisfied).

        Algorithm:
        ----------
        1 - Create a list with goals connected to activated P-nodes.
        2 - Append to that list those goals that have a value function whose first element is in that list.

        Parameters:
        ----------
        sensing : {sensor_name: [{attribute: value, ...}, ...], ...}
            A sensorial state.

        Returns:
        --------
        feasible_goals : [goal, ...]
            List of goals that can be satisfied, totally or partially.

        """
        activated_pnodes = (p_node for p_node in self.p_nodes if max(p_node.activation) > 0.0)
        feasible_goals = [
            neighbor for p_node in activated_pnodes for neighbor in p_node.neighbors if neighbor.type == "Goal"
        ]
        feasible_goals.extend(
            (
                goal
                for goal in self.goals
                for value_function in goal.value_functions
                for sub_goal in value_function
                if sub_goal in feasible_goals
            )
        )
        return feasible_goals

    def random_policy(self):
        """Select a random policy. In order to avoid problems with random numbers' generation, we use a pool."""
        if self.policies_to_test == []:
            self.policies_to_test = copy(self.policies)
        policy = self.policies_to_test[numpy.random.randint(len(self.policies_to_test))]
        return policy

    def select_policy(self, sensing):
        """
        Select the policy to be run.

        Algorithm:
        ----------
        1 - Find the policy with the largest activation.
        2 - If 1 fails, do prospection to generate new VFs and try to find a meaningful policy.
        3 - If 2 fails, activate the last goal that obtained reward.
        4 - If 3 fails, activate a random (and with a feasible start) goal.
        5 - If 4 fails, just pick a policy randomly.

        Notes:
        ------
        - Steps 3 and 4 only work if the "end" parameter for the selected goal is NOT present in the configuration file.
        Otherwise, update_activations() will overwrite the change in the goal's activation!

        Parameters:
        ----------
        sensing : {sensor_name: [{attribute: value, ...}, ...], ...}
            A sensorial state.

        Returns:
        --------
        policy : Policy
            Selected policy

        """
        self.update_activations(sensing, new_sensings=True)
        # 1
        policy = max(self.policies, key=attrgetter("activation"))
        # 2
        if not policy.activation:
            if self.do_prospection():
                self.update_activations(sensing, new_sensings=False)
                policy = max(self.policies, key=attrgetter("activation"))
        if self.goals:
            # 3
            if not policy.activation:
                goal = max(self.goals, key=attrgetter("reward"))
                if goal.reward > goal.threshold:
                    goal.new_activation = 1.0
                    self.update_activations(sensing, new_sensings=False)
                    policy = max(self.policies, key=attrgetter("activation"))
            # 4
            if not policy.activation:
                feasible_goals = self.get_feasible_goals()
                if feasible_goals:
                    goal = random.choice(feasible_goals)
                    goal.new_activation = 1.0
                    self.update_activations(sensing, new_sensings=False)
                    policy = max(self.policies, key=attrgetter("activation"))
        # 5
        if not policy.activation:
            policy = self.random_policy()
        rospy.loginfo("Selecting a policy => " + policy.ident + " (" + str(policy.activation) + ")")
        return policy

    @staticmethod
    def check_perception_sanity(policy, previous_state):
        """
        Check if there is not perception ambiguity.

        If there is not perception in the policy, this means that the P-node was not activated, (see
        CNode.update_activation()). In this case, if there is some multi-valuated sensor, we log a warning, as this is
        not solved yet. Policies should be parametizer and, if a policy is randomly executed, it should be called
        several times, once for each combination of sensor values, so we would know what perception has to be added to
        the P-node. This would simplify also the simulator and the preprogrammed goals (that should dissapear, anyway).

        Parameters:
        ----------
        policy: Policy
            Executed policy.
        previous_state : {sensor_name: [{attribute: value, ...}, ...], ...}
            Sensorial state before the execution of the policy.

        Returns:
        --------
        bool
            Is known what sensor values were employed by the policy?

        """
        problem = False
        if not policy.perception:
            if max([len(sensor) for sensor in previous_state.values()]) == 1:
                policy.perception = OrderedDict()
                for sensor, value in previous_state.items():
                    policy.perception[sensor + "0"] = value[0]
        if not policy.perception:
            rospy.logwarn(
                "It can't be determined what combination of sensor values caused a goal success, "
                "so it is not possible to add a point / create a new P-node"
            )
            problem = True
        return not problem

    @staticmethod
    def create_perception(sensing):
        """Create a new state space with one point."""
        perception = OrderedDict()
        for sensor, values in sensing.items():
            for idx, value in enumerate(values):
                perception[sensor + str(idx)] = value
        return perception

    def create_space(self, sensing):
        """Create a new state space with one point."""
        if not sensing:
            rospy.logerr("Error: creating an empty state space!!!")
        space = self.class_from_classname(self.default_class["Space"])()
        space.add_point(self.create_perception(sensing), 1.0)
        return space

    @staticmethod
    def add_point(p_node, perception):
        """Add a point to the p-node."""
        p_node.add_perception(perception, 1.0)
        rospy.loginfo("Added point in p-node " + p_node.ident)

    @staticmethod
    def add_antipoint(p_node, perception):
        """Add an anti-point to the p-node."""
        p_node.add_perception(perception, -1.0)
        rospy.loginfo("Added anti-point in p-node " + p_node.ident)

    def update_pnodes(self, current_state):
        """
        Update P-nodes according to the accomplishment of the connected goal.

        Algorithm:
        ----------
        1 - Select all the C-nodes connected to the executed policy and to activated forward models.
        2 - For each one of those C-nodes, if the goal connected to it is contained in the new state (so it was
        satisfied), then add the previous state as a point to the P-node connected to it.
        3 - ... otherwise, add it as anti-point.

        Notes:
        ------
        - The underlying implementation of P-nodes is in charge of points and anti-points processing. For instance,
        when adding and anti-point with a goal that was not activated, if the P-node is a neural network, the
        anti-point could be used in training, but if the P-node is just a point-based space, the anti-point could be
        discarded.
        - Right now, LTM is only storing causal relationships that lead to a reward. Another option would be to store
        every discovered causal relationship. This would simplify update_goals() as part of its process could be done
        here, but memory requirements would be increase. We need to think about it...
        - It is assumed that the active forward model doesn't change in a cycle. This is a huge simplification, be
        ware!!!
        - We are not storing the particular subset of sensing used by a policy in every instant of time so, when we
        inspect FM's activation, we just pick the maximum. This is clearly something that needs to be changed.

        Parameters:
        ----------
        current_state : (old_sensing, policy, sensing)
            old_sensing : {sensor_name: [{attribute: value, ...}, ...], ...}
                Sensorial state before the execution of the policy.
            policy: Policy
                Executed policy.
            sensing : {sensor_name: [{attribute: value, ...}, ...], ...}
                Sensorial state after the execution of the policy.

        """
        old_sensing, policy, sensing = current_state
        for c_node in (node for node in policy.neighbors if node.type == "CNode"):
            if c_node.forward_model.max_activation > c_node.forward_model.threshold:
                if self.check_perception_sanity(policy, old_sensing):
                    if self.create_space(sensing).contains(c_node.goal.space):
                        self.add_point(c_node.p_node, policy.perception)
                    else:
                        self.add_antipoint(c_node.p_node, policy.perception)

    def read_reward(self):
        """Return the last received reward."""
        rospy.loginfo("Reading reward...")
        self.reward_event.wait()
        self.reward_event.clear()
        return self.received_reward

    def new_cnode(self, perception, goal, policy, candidate_cnode=None):
        """
        Create a new C-node and its corresponding P-node.

        Notes:
        -----
        - It is assumed that the active forward model doesn't change in a cycle. This is a huge simplification, be
        ware!!!

        """
        space = candidate_cnode.p_node.space.specialize() if candidate_cnode else None
        p_node = self.add_node(
            node_type=bytes_to_native_str(b"PNode"),
            class_name=self.default_class["PNode"],
            ros_node_prefix=self.default_ros_node_prefix["PNode"],
            ros_data_prefix=self.default_ros_data_prefix["PNode"],
            space_class=self.default_class["Space"],
            space=space,
        )
        p_node.add_perception(perception, 1.0)
        forward_model = max(self.forward_models, key=attrgetter("max_activation"))
        neighbors = [p_node, forward_model, goal, policy]
        c_node = self.add_node(
            node_type=bytes_to_native_str(b"CNode"),
            class_name="mdb_ltm.cnode.CNode",
            ros_node_prefix=self.default_ros_node_prefix["CNode"],
            ros_data_prefix=self.default_ros_data_prefix["CNode"],
            neighbors=neighbors,
            weight=1.0,
        )
        rospy.loginfo("Added point in p-node " + p_node.ident)
        rospy.loginfo(
            "New c-node "
            + c_node.ident
            + " joining "
            + p_node.ident
            + ", "
            + forward_model.ident
            + ", "
            + goal.ident
            + " and "
            + policy.ident
        )

    @staticmethod
    def find_changes(relevant_sensors, sensing1, sensing2):
        """Detect what sensors changed their values between rewards."""
        for sensor in sensing1:
            if sensor not in sensing2:
                relevant_sensors[sensor] = sensing1[sensor]
            else:
                if sensor in relevant_sensors:
                    changed_objects = relevant_sensors[sensor]
                else:
                    changed_objects = []
                for sensed_object in sensing1[sensor]:
                    if (sensed_object not in sensing2[sensor]) and (sensed_object not in changed_objects):
                        changed_objects.append(sensed_object)
                for sensed_object in sensing2[sensor]:
                    if (sensed_object not in sensing1[sensor]) and (sensed_object not in changed_objects):
                        changed_objects.append(sensed_object)
                if changed_objects:
                    relevant_sensors[sensor] = changed_objects
        for sensor in sensing2:
            if sensor not in sensing1:
                relevant_sensors[sensor] = sensing2[sensor]

    @staticmethod
    def filter_sensing(sensing, sensing_filter):
        """Prune every sensorization from sensing not present also in sensing_filter."""
        pruned_sensing = OrderedDict()
        for sensor in sensing:
            if sensor in sensing_filter:
                new_sensing = []
                for sensed_object in sensing[sensor]:
                    if sensed_object in sensing_filter[sensor]:
                        new_sensing.append(sensed_object)
                if new_sensing:
                    pruned_sensing[sensor] = new_sensing
        return pruned_sensing

    def filter_stm(self, stm, reset_sensing):
        """Prune every sensorization that didn't change."""
        new_stm = []
        relevant_sensors = OrderedDict()
        for (pre_sensing, policy, post_sensing) in stm:
            self.find_changes(relevant_sensors, pre_sensing, post_sensing)
        _, _, last_sensing = stm[-1]
        self.find_changes(relevant_sensors, last_sensing, reset_sensing)
        if not relevant_sensors:
            rospy.logerr("Error: not relevant sensors found when filtering STM!!!")
        for (old_sensing, policy, sensing) in stm:
            new_stm.append(
                (
                    self.filter_sensing(old_sensing, relevant_sensors),
                    policy,
                    self.filter_sensing(sensing, relevant_sensors),
                )
            )
        return new_stm

    def update_goals(self, stm, last_sensing):
        """
        Assign reward to every goal contained in the current state. A new goal is created if necessary.

        Algorithm:
        ----------
        1 - For each element in STM:
            1.1 - Check if there is a goal connected to the policy exactly matching the current state.
            1.2 - Otherwise, check if there is a goal connected to the policy defined over the same sensors but that
            does not include the current state. In this case, add the current state to the goal and the previous state
            to the P-node. WARNING: this assumes that the same policy cannot be used for two different things that
            need the same sensors and for achieving the same goal.
            1.3 - Otherwise, check if there is a goal NOT connected to the policy matching the current state and, in
            that case, create a new C-node connecting policy and goal.
            1.4 - Otherwise, check if there is a goal connected to the policy embeded in the current state and, in that
            case, create a new specialized goal and connect it to the policy.
            1.5 - Otherwise, create a new goal and connect it to the policy.
            1.6 - Add the goal to a candidate VF.
        2 - Assign reward to the matching goal of the latest instant time (the one when reward was obtained).
        3 - Add the candidate VF to that matching goal if its length is larger than one.

        Notes:
        -----
        - Points and anti-points are not managed in this method because it is done in update_pnodes(). First point of
        this algorithm could be done also in update_pnodes() if we stored every causal relationship and not only those
        that lead to a reward.
        - At this time, only the goal that matches the current state receives reward. Another possibility would be to
        assign reward to every goal embeded in the current state.
        - We are not storing the particular subset of sensing used by a policy in every instant of time so, when we
        inspect FM's activation, we just pick the maximum. This is clearly something that needs to be changed.
        - VERY IMPORTANT WARNING: This assumes that the goal did not change during the whole STM contents.

        Parameters:
        ----------
        stm : [(old_sensing, policy, sensing), ...]
            old_sensing : {sensor_name: [{attribute: value, ...}, ...], ...}
                Sensorial state before the execution of the policy.
            policy: Policy
                Executed policy.
            sensing : {sensor_name: [{attribute: value, ...}, ...], ...}
                Sensorial state after the execution of the policy.
        first_sensing : {sensor_name: [{attribute: value, ...}, ...], ...}
            First sensorial state just after the previous reward.
        last_sensing : {sensor_name: [{attribute: value, ...}, ...], ...}
            Sensorial state after receiving reward.

        """
        rospy.loginfo("Reward obtained => updating goals, c-nodes and p-nodes")
        rospy.loginfo("STM =>")
        rospy.loginfo(stm)
        stm = self.filter_stm(stm, last_sensing)
        rospy.loginfo("Filtered STM =>")
        rospy.loginfo(stm)
        candidate_vf = []
        # 1
        for (old_sensing, policy, sensing) in stm:
            old_perception = self.create_perception(old_sensing)
            perception = self.create_perception(sensing)
            space = self.create_space(sensing)
            perfect_goal = None
            # 1.1
            for cnode in (node for node in policy.neighbors if node.type == "CNode"):
                if cnode.forward_model.max_activation > cnode.forward_model.threshold:
                    if cnode.goal.space.contains(space) and cnode.goal.space.same_sensors(space):
                        perfect_goal = cnode.goal
                        break
            # 1.2
            if not perfect_goal:
                for cnode in (node for node in policy.neighbors if node.type == "CNode"):
                    if cnode.forward_model.max_activation > cnode.forward_model.threshold:
                        if cnode.goal.space.same_sensors(space):
                            cnode.goal.space.add_point(perception, 1.0)
                            self.add_point(cnode.p_node, old_perception)
                            perfect_goal = cnode.goal
                            break
            # 1.3
            if not perfect_goal:
                for goal in self.goals:
                    if goal.space.contains(space) and goal.space.same_sensors(space):
                        perfect_goal = goal
                        self.new_cnode(old_perception, perfect_goal, policy)
                        break
            # 1.4
            if not perfect_goal:
                for cnode in (node for node in policy.neighbors if node.type == "CNode"):
                    if cnode.forward_model.max_activation > cnode.forward_model.threshold:
                        if cnode.goal.space.contains(space):
                            space = cnode.goal.space.specialize(self.create_perception(sensing))
                            perfect_goal = self.add_node(
                                node_type=bytes_to_native_str(b"Goal"),
                                class_name="mdb_ltm.goal.Goal",
                                ros_node_prefix=self.default_ros_node_prefix["Goal"],
                                ros_data_prefix=self.default_ros_data_prefix["Goal"],
                                space_class=self.default_class["Space"],
                                space=space,
                            )
                            self.new_cnode(old_perception, perfect_goal, policy, cnode)
                            break
            # 1.5
            if not perfect_goal:
                perfect_goal = self.add_node(
                    node_type=bytes_to_native_str(b"Goal"),
                    class_name="mdb_ltm.goal.Goal",
                    ros_node_prefix=self.default_ros_node_prefix["Goal"],
                    ros_data_prefix=self.default_ros_data_prefix["Goal"],
                    space_class=self.default_class["Space"],
                    space=space,
                )
                self.new_cnode(old_perception, perfect_goal, policy)
            # 1.6
            candidate_vf.append(perfect_goal)
        # 2
        perfect_goal.reward = self.current_reward
        rospy.loginfo("Successful goal " + perfect_goal.ident + " => " + str(self.current_reward))
        # 3
        if len(candidate_vf) > 1:
            perfect_goal.add_value_function(candidate_vf)

    def update_policies_to_test(self, policy=None):
        """Maintenance tasks on the pool of policies used to choose one randomly when needed."""
        if policy:
            if policy in self.policies_to_test:
                self.policies_to_test.remove(policy)
        else:
            self.policies_to_test = copy(self.policies)

    @staticmethod
    def graph_set_edge_properties(edges):
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

    def update_status(self):
        """Update experiment information, and write / publish it."""
        for file_object in self.files:
            file_object.write()
        self.info_publisher.publish(
            iteration=self.iteration,
            current_world=self.current_world,
            current_policy=self.current_policy,
            current_reward=self.current_reward,
            current_goal=self.current_goal,
        )

    def reset_world(self):
        """Reset the world if necessary, according to the experiment parameters."""
        changed = False
        self.trial += 1
        if self.trial == self.trials or self.current_reward >= 0.9:
            self.trial = 0
            changed = True
        if ((self.iteration % self.period) == 0) or self.restoring:
            self.current_world = self.worlds[(self.iteration // self.period) % len(self.worlds)]
            self.trial = 0
            self.restoring = False
            changed = True
        if changed:
            rospy.loginfo("Asking for a world reset...")
            self.control_publisher.publish(
                command="reset_world", world=self.current_world, reward=(self.current_reward >= 0.9)
            )
        return changed

    def run(self, seed=None, log_level="INFO"):
        """Start the LTM part of the brain."""
        try:
            self.setup(log_level, seed)
            rospy.loginfo("Running LTM...")
            sensing = self.read_perceptions()
            stm = []
            while (not rospy.is_shutdown()) and (self.iteration <= self.iterations):
                if not self.paused:
                    rospy.loginfo("*** ITERATION: " + str(self.iteration) + " ***")
                    self.current_policy = self.select_policy(sensing)
                    self.current_policy.execute()
                    old_sensing, sensing = sensing, self.read_perceptions()
                    current_state = (old_sensing, self.current_policy, sensing)
                    self.update_pnodes(current_state)
                    if self.sensorial_changes():
                        stm.append(current_state)
                    self.current_reward = self.read_reward()
                    self.current_goal = None
                    if self.reset_world():
                        reset_sensing = self.read_perceptions()
                        if self.current_reward >= 0.9:
                            self.update_goals(stm, reset_sensing)
                            self.current_goal = max(self.goals, key=attrgetter("reward"))
                        sensing = reset_sensing
                        stm = []
                    self.update_policies_to_test(policy=self.current_policy if not self.sensorial_changes() else None)
                    self.update_status()
                    self.iteration += 1
        except rospy.ROSInterruptException:
            rospy.logerr("Exception caught! Or you pressed CTRL+C or something went wrong...")
