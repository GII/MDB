import sys
import rclpy
from rclpy.node import Node
from operator import attrgetter
import random
import yaml
import threading
import numpy
from copy import copy

from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

#from core_interfaces.srv import SendToLTM

from core.service_client import ServiceClient
from cognitive_node_interfaces.srv import Execute, GetActivation, IsReached, GetInformation, AddPoint
from cognitive_node_interfaces.msg import Perception
from core_interfaces.srv import GetNodeFromLTM, CreateNode
from core_interfaces.msg import ControlMsg

from core.cognitive_node import CognitiveNode




class MainLoop(Node):
    """
    MainLoop class for managing the main loop of the system.

    This class handles the core logic of the system, including reading perceptions,
    selecting policies, and executing policies.
    """

    def __init__(self,name, **params):
        """
        Constructor for the MainLoop class.

        Initializes the MainLoop node and starts the main loop execution.
        """        
        super().__init__(name)
        self.iteration = 1
        self.iterations= 0
        self.trials=0
        self.trial=0
        self.period=0
        self.current_policy = None
        self.paused=False
        self.stop=False
        self.LTM_id = "" #id of LTM currently being run by cognitive loop

        # List of dics, like [{"name": "pnode1", "node_type": "PNode", "activation": 0.0}, {"name": "cnode1", "node_type": "CNode", "activation": 0.0}]
        self.LTM_cache=[]
        self.perception_suscribers={}
        self.perception_cache={}
        self.reward_threshold= 0.9
        self.subgoals=False
        self.policies_to_test=[]

        self.control_publisher= self.create_publisher(ControlMsg, 'main_loop/control', 10)



        self.perception_callback_group=MutuallyExclusiveCallbackGroup()
        self.services_callback_group=MutuallyExclusiveCallbackGroup()


        for key, value in params.items():
            self.get_logger().debug('Setting atribute: ' + str(key) + ' with value: ' + str(value))
            setattr(self, key, value)

        #Read LTM and configure perceptions
        self.read_ltm()
        self.configure_perceptions()

        loop_thread = threading.Thread(target=self.run, daemon=True)
        loop_thread.start()
    
    def configure_perceptions(self): #TODO(efallash): Add condition so that perceptions that are already included do not create a new suscription. For the case that new perceptions are added to the LTM and only some perceptions need to be configured
        self.get_logger().info('Configuring perceptions...')
        perceptions=[perception for perception in self.LTM_cache if perception['node_type']=='Perception']

        self.get_logger().info(str(perceptions))

        for perception_dict in perceptions:
            perception=perception_dict['name']

            subscriber=self.create_subscription(Perception, f'/perception/{perception}/value', self.receive_perception_callback, 1, callback_group= self.perception_callback_group)
            self.get_logger().debug(f'Subscription to: /perception/{perception}/value created')
            self.perception_suscribers[perception]=subscriber
            self.perception_cache[perception]={}
            self.perception_cache[perception]['flag']=threading.Event()
        #TODO check that all perceptions in the cache still exist in the LTM and destroy suscriptions that are no longer used



    def read_perceptions(self):
        self.get_logger().info('Reading perceptions...')

        sensing={}

        for sensor in self.perception_cache.keys(): #TODO: Consider perception activation when reading
            self.perception_cache[sensor]['flag'].wait()
            sensing[sensor]=self.perception_cache[sensor]['data']
            self.perception_cache[sensor]['flag'].clear()

        self.get_logger().debug('Perceptions: '+str(sensing))
        return sensing

    def receive_perception_callback(self, msg):
        perception_dict=CognitiveNode.perception_msg_to_dict(msg)
        
        for sensor in perception_dict.keys():
            if sensor in self.perception_cache:
                self.perception_cache[sensor]['data']=perception_dict[sensor]
                self.perception_cache[sensor]['flag'].set() 
                self.get_logger().debug(f'Receiving perception: {sensor} ...')
            else:
                self.get_logger().error('Received sensor not registered in local perception cache!!!')
            

    def read_ltm(self):
        self.get_logger().info('Reading nodes from LTM: '+ self.LTM_id + '...')

        #Call get_node service from LTM
        service_name = '/' + str(self.LTM_id) + '/get_node'
        request=""
        client = ServiceClient(GetNodeFromLTM, service_name)
        ltm_response = client.send_request(name=request)

        

        client.destroy_node()
        #Process data string
        ltm_cache=yaml.safe_load(ltm_response.data)

        self.get_logger().info(str(ltm_cache))

        for node_type in ltm_cache.keys():
            for node in ltm_cache[node_type].keys():
                self.LTM_cache.append({'name': node, 'node_type': node_type, 'activation': ltm_cache[node_type][node]['activation']})

        self.get_logger().info(str(self.LTM_cache))
        return None

    def ltm_change_callback(self):
        self.get_logger().info('Processing change from LTM...') # TODO(efallash): implement
        pass


    def select_policy(self, sensing):
        """
        Selects the policy with the higher activation based on the current sensing.

        :param sensing: The current sensing.
        :type sensing: Any
        :return: The selected policy.
        :rtype: str
        """
        self.update_activations(sensing)

        policy_activations={}
        for node in self.LTM_cache:
            if node['node_type']=='Policy':
                policy_activations[node['name']]=node['activation']

        self.get_logger().debug('Select_policy - Activations: '+ str(policy_activations))

        policy = max(zip(policy_activations.values(), policy_activations.keys()))[1]

        if not policy_activations[policy]:
            policy= self.random_policy()

        self.get_logger().info(f"Selecting a policy => {policy} ({policy_activations[policy]})" )


        return policy
    
    def random_policy(self):
        """
        Selects random policy.
        """

        if self.policies_to_test==[]:
            self.policies_to_test = [node['name'] for node in self.LTM_cache if node['node_type']=='Policy']
        
        policy = random.choice(self.policies_to_test)

        return policy
    
    def update_policies_to_test(self, policy=None):
        """Maintenance tasks on the pool of policies used to choose one randomly when needed."""


        if policy:
            if policy in self.policies_to_test:
                self.policies_to_test.remove(policy)
        else:
            self.policies_to_test = [node['name'] for node in self.LTM_cache if node['node_type']=='Policy']

    def sensorial_changes(self, sensing, old_sensing):
        """Return false if all perceptions have the same value as the previous step. True otherwise."""

        for sensor in sensing:
            for perception, perception_old in zip(sensing[sensor], old_sensing[sensor]):
                if isinstance(perception, dict):
                    for attribute in perception:
                        difference = abs(perception[attribute] - perception_old[attribute])
                        if difference > 0.01:
                            self.get_logger().debug('Sensorial change detected')
                            return True
                else:
                    if abs(perception[0] - perception_old[0]) > 0.01:
                        self.get_logger().debug('Sensorial change detected')
                        return True
        self.get_logger().debug('No sensorial change detected')
        return False
    
    def update_activations(self, perception, new_sensings=True): 
        self.get_logger().info('Updating activations...')

        for node in self.LTM_cache:
            if node['node_type']== 'PNode':
                if new_sensings:
                    activation=self.request_activation(node['name'], perception)
                    node['activation']=activation
            else:
                activation=self.request_activation(node['name'], perception)
                node['activation']=activation

        self.get_logger().info(str(self.LTM_cache))






    def request_activation(self, name, sensing):
        service_name = 'cognitive_node/' + str(name) + '/get_activation'
        activation_client = ServiceClient(GetActivation, service_name)
        perception = CognitiveNode.perception_dict_to_msg(sensing)
        activation = activation_client.send_request(perception = perception)
        activation_client.destroy_node()
        return activation.activation
        
    def request_neighbors(self, name):
        service_name = 'cognitive_node/' + str(name) + '/get_information'
        information_client=ServiceClient(GetInformation, service_name)
        information=information_client.send_request()

        neighbors_names=information.neighbors_name
        neighbors_types=information.neighbors_type

        neighbors = [{'name': node[1], 'node_type': node[2]} for node in zip(neighbors_names,neighbors_types)]

        return neighbors

    def execute_policy(self, policy):
        """
        Execute a policy.

        This method sends a request to the policy to be executed.

        :param policy: The policy to execute.
        :type policy: str
        :return: The response from executing the policy.
        :rtype: The executed policy.
        """
        self.get_logger().info('Executing policy ' + str(policy)+ '...')

        service_name = 'policy/' + str(policy) + '/execute'
        client = ServiceClient(Execute, service_name)
        policy_response = client.send_request()
        client.destroy_node()
        return policy_response.policy
    
    def get_current_goal(self):
        self.get_logger().info('Selecting goal with higher activation...')

        goal_activations={}
        for node in self.LTM_cache:
            if node['node_type']=='Goal':
                goal_activations[node['name']]=node['activation']


        self.get_logger().info('Select__current_goal - Activations: '+ str(goal_activations))

        goal = max(zip(goal_activations.values(), goal_activations.keys()))[1]
        
        return goal
    
    def get_current_reward(self):
        self.get_logger().info('Reading reward...')

        service_name = 'goal/' + str(self.current_goal) + '/is_reached'
        reward_client = ServiceClient(IsReached, service_name)
        reward = reward_client.send_request()
        reward_client.destroy_node()

        self.get_logger().info(f'get_current_reward - Reward {reward.reached}')

        return reward.reached
    
    def get_current_world_model(self):
        self.get_logger().info('Selecting world model with higher activation...')

        WM_activations={}
        for node in self.LTM_cache:
            if node['node_type']=='WorldModel':
                WM_activations[node['name']]=node['activation']


        self.get_logger().info('Select__current_goal - Activations: '+ str(WM_activations))

        WM = max(zip(WM_activations.values(), WM_activations.keys()))[1]
        
        return WM

    def get_max_activation_node (self, node_type): #TODO: Refactor
        pass

    def update_pnodes_reward_basis(self, perception, policy, goal, reward):
        self.get_logger().info('Updating p-nodes/c-nodes...')
        policy_neighbors=self.request_neighbors(policy)
        cnodes=[node['name'] for node in policy_neighbors if node['node_type']=='CNode']
        threshold=0.1

        for cnode in cnodes:
            cnode_neighbors=self.request_neighbors(cnode)
            world_model=next((name for name, node_type in cnode_neighbors.items() if node_type=='WorldModel'))
            goal=next((name for name, node_type in cnode_neighbors.items() if node_type=='Goal'))
            pnode=next((name for name, node_type in cnode_neighbors.items() if node_type=='PNode'))

            world_model_activation= next([node['activation'] for node in self.LTM_cache if node['name']==world_model])
            goal_activation= next([node['activation'] for node in self.LTM_cache if node['name']==goal])
            pnode_activation= next([node['activation'] for node in self.LTM_cache if node['name']==pnode])


            if (
                world_model_activation>threshold 
                and goal_activation>threshold
            ):
                if reward>threshold:
                    self.add_point(pnode, perception)
                elif pnode_activation>threshold:
                    self.add_antipoint(pnode, perception)

        if (not cnodes) and (reward > threshold):
            self.new_cnode(perception,goal,policy)

    def add_point(self, name, sensing):
        service_name = 'pnode/' + str(name) + '/add_point'
        add_point_client = ServiceClient(AddPoint, service_name)
        perception = CognitiveNode.perception_dict_to_msg(sensing)

        response = add_point_client.send_request(point= perception, confidence=1.0)
        add_point_client.destroy_node()

        self.get_logger().info(f'Added point in pnode {name}: {str(perception)}')

        return response.added
        

    def add_antipoint(self, name, sensing):
        service_name = 'pnode/' + str(name) + '/add_point'
        add_point_client = ServiceClient(AddPoint, service_name)
        perception = CognitiveNode.perception_dict_to_msg(sensing)

        response = add_point_client.send_request(point= perception, confidence=-1.0)
        add_point_client.destroy_node()

        self.get_logger().info(f'Added point in pnode {name}: {str(perception)}')
        
        return response.added

    def new_cnode(self, perception, goal, policy):
        self.get_logger().info('Creating Cnode...')
        world_model = self.get_current_world_model()
        ident=f'{world_model}__{goal}__{policy}'

        #TODO: Obtain class names from config file
        space_class = 'cognitive_nodes.space.PointBasedSpace' 
        pnode_class = 'cognitive_nodes.pnode.PNode' 
        cnode_class = 'cognitive_nodes.pnode.CNode' 
        
        pnode_name=f'pnode_{ident}'
        pnode = self.create_node_client(
            name=pnode_name, 
            class_name=pnode_class, 
            parameters={'space_class': space_class}
            )
        if not pnode:
            self.get_logger().fatal(f'Failed creation of PNode {pnode_name}')
        self.add_point(pnode_name, perception)

        neighbors={'neighbors': [node for node in self.LTM_cache if node['name'] in [world_model,goal,policy,pnode]]}

        cnode_name=f'cnode_{ident}'
        cnode= self.create_node_client(
            name=cnode_name,
            class_name=cnode_class,
            parameters=str(neighbors)
        )

        if not pnode:
            self.get_logger().fatal(f'Failed creation of CNode {cnode_name}')

        #Update LTMCache with new CNode/PNode. This is a HACK, should be integrated with LTM's changes topic
        self.LTM_cache.append({'name': pnode_name, 'node_type': 'PNode', 'activation': 0})
        self.LTM_cache.append({'name': cnode_name, 'node_type': 'CNode', 'activation': 0})

    def create_node_client(self, name, class_name, parameters={}):
        self.get_logger().info('Requesting node creation')

        service_name = 'commander/create'
        client = ServiceClient(CreateNode, service_name)
        response = client.send_request()
        client.destroy_node()

        return response.created        

    def reset_world(self):
        """Reset the world if necessary, according to the experiment parameters."""
        changed = False
        self.trial += 1
        if self.trial == self.trials or self.current_reward > 0.9:
            self.trial = 0
            changed = True
        if ((self.iteration % self.period) == 0):
            #TODO: Implement periodic world changes
            pass
        if changed:
            self.get_logger().info("Asking for a world reset...")
            msg=ControlMsg()
            msg.command="reset_world"
            msg.world=self.current_world
            msg.iteration=self.iteration
            self.control_publisher.publish(msg)
        return changed

    def update_status(self): #TODO(efallash): implement
        self.get_logger().info('Writing files publishing status...')
        pass







    def run(self): 
        """
        Run the main loop of the system.
        """

        self.get_logger().info('Running MDB with LTM:' + str(self.LTM_id))

        sensing = self.read_perceptions()
        stm=[]
        while (self.iteration<=self.iterations) and (not self.stop): # TODO: check conditions to continue the loop

            if not self.paused:
            
                self.get_logger().info('*** ITERATION: ' + str(self.iteration) + '/' +str(self.iterations) + ' ***')
                
                self.current_policy = self.select_policy(sensing)
                self.execute_policy(self.current_policy)
                old_sensing, sensing = sensing, self.read_perceptions()

                if not self.subgoals:
                    self.current_goal = self.get_current_goal()
                    self.current_reward=self.get_current_reward()
                    self.update_pnodes_reward_basis()
                else:
                    raise NotImplementedError #TODO: Implement prospection methods
                

                if self.reset_world():
                    reset_sensing=self.read_perceptions()
                    
                    if self.current_reward > self.reward_threshold and self.subgoals:
                        raise NotImplementedError #TODO: Implement prospection methods
                    
                    sensing= reset_sensing

                self.update_policies_to_test(
                        policy=self.current_policy if not self.sensorial_changes(sensing, old_sensing) else None
                    )

                self.update_status()
                self.iteration += 1

            
    




        

def main(args=None):
    rclpy.init()

    #TESTING ONLY
    params={'iterations':10000, 'trials':10, 'LTM_id':'ltm_0', 'subgoals':False}

    #executor=MultiThreadedExecutor(num_threads=2)
    executor=SingleThreadedExecutor()
    main_loop = MainLoop('main_loop',**params)

    executor.add_node(main_loop) #Test
    main_loop.get_logger().info('Runnning node')

    try:
        executor.spin()
    except KeyboardInterrupt:
        main_loop.destroy_node()




if __name__ == '__main__':
    main()