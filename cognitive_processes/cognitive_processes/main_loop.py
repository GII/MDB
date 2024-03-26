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
from cognitive_node_interfaces.srv import Execute, GetActivation
from cognitive_node_interfaces.msg import Perception
from core_interfaces.srv import GetNodeFromLTM

from core.cognitive_node import CognitiveNode




class MainLoop(Node):
    """
    MainLoop class for managing the main loop of the system.

    This class handles the core logic of the system, including reading perceptions,
    selecting policies, and executing policies.
    """

    def __init__(self,name, **params): #TODO(efallash): Pass parameters to this constructor (iterations, trials, options)
        """
        Constructor for the MainLoop class.

        Initializes the MainLoop node and starts the main loop execution.
        """        
        super().__init__(name)
        self.iteration = 0
        self.iterations= 0
        self.trials=0
        self.current_policy = None
        self.paused=False
        self.stop=False
        self.LTM_id = "" #id of LTM currently being run by cognitive loop
        self.LTM_cache= {'Drive': [], 'Goal': [], 'Need': [], 'Policy': [], 'Perception': [],'PNode': [], 'UtilityModel': [], 'WorldModel': []}
        self.perception_suscribers={}
        self.perception_cache={}
        self.reward_threshold= 0.9
        self.subgoals=False
        self.policies_to_test=[]



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

    # def send_request_to_LTM(self, command): #TODO(efallash): Modify LTM service so that this method makes sense
    #     """
    #     Send a request to the LTM.

    #     :param command: The command to send.
    #     :type command: str
    #     :return: The response from the LTM.
    #     :rtype: core_interfaces.srv.SendToLTM_Response
    #     """
    #     service_name = 'send_to_LTM'
    #     send_to_LTM_client = ServiceClient(SendToLTM, service_name)
    #     ltm_response = send_to_LTM_client.send_request(command=command)
    #     send_to_LTM_client.destroy_node()
    #     return ltm_response
    
    def configure_perceptions(self): #TODO(efallash): Add condition so that perceptions that are already included do not create a new suscription. For the case that new perceptions are added to the LTM and only some perceptions need to be configured
        self.get_logger().info('Configuring perceptions...')
        for perception in self.LTM_cache['Perception']:
            subscriber=self.create_subscription(Perception, f'/perception/{perception}/value', self.receive_perception_callback, 1, callback_group= self.perception_callback_group)
            self.get_logger().debug(f'Subscription to: /perception/{perception}/value created')
            self.perception_suscribers[perception]=subscriber
            self.perception_cache[perception]={}
            self.perception_cache[perception]['flag']=threading.Event()
        #TODO check that all perceptions in the cache still exist in the LTM and destroy suscriptions that are no longer used



    def read_perceptions(self):
        self.get_logger().info('Reading perceptions...')

        sensing={}

        for sensor in self.perception_cache.keys():
            self.perception_cache[sensor]['flag'].wait()
            sensing[sensor]=self.perception_cache[sensor]['data']
            self.perception_cache[sensor]['flag'].clear()

        self.get_logger().info(str(sensing))
        return sensing

    def receive_perception_callback(self, msg):
        perception_dict=CognitiveNode.perception_msg_to_dict(msg)
        
        for sensor in perception_dict.keys():
            if sensor in self.perception_cache:
                self.perception_cache[sensor]['data']=perception_dict[sensor]
                self.perception_cache[sensor]['flag'].set() 
                self.get_logger().info(f'Receiving perception: {sensor} ...')
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

        #Save dict with list of nodes in LTM cache
        for node_type in ltm_cache.keys():
            self.LTM_cache[node_type]= [*ltm_cache[node_type]]        
        return None

    def ltm_change_callback(self):
        self.get_logger().info('Processing change from LTM...') # TODO(efallash): implement
        pass


    def select_policy(self, sensing): # TODO: implement
        """
        Selects the policy with the higher activation based on the current sensing.

        :param sensing: The current sensing.
        :type sensing: Any
        :return: The selected policy.
        :rtype: str
        """
        
        policy_activations={}
        for policy in self.LTM_cache['Policy']:
            service_name = 'cognitive_node/' + str(policy) + '/get_activation'
            activation_client = ServiceClient(GetActivation, service_name)
            perception = CognitiveNode.perception_dict_to_msg(sensing)
            activation = activation_client.send_request(perception = perception)
            activation_client.destroy_node()
            policy_activations[policy]=activation.activation

        self.get_logger().info('Test select_policy - Activations: '+ str(policy_activations))

        policy = max(zip(policy_activations.values(), policy_activations.keys()))[1]

        self.get_logger().info('Test select_policy - Selected Policy: ' + str(policy))

        if not policy_activations[policy]:
            policy= self.random_policy()

        self.get_logger().info(f"Selecting a policy => {policy} ({policy_activations[policy]})" )


        return policy
    
    def random_policy(self):
        """
        Selects random policy.
        """

        if self.policies_to_test==[]:
            self.policies_to_test = copy(self.LTM_cache['Policy'])
        
        policy = random.choice(self.policies_to_test)

        return policy
    
    def update_policies_to_test(self, policy=None):
        """Maintenance tasks on the pool of policies used to choose one randomly when needed."""


        if policy:
            if policy in self.policies_to_test:
                self.policies_to_test.remove(policy)
        else:
            self.policies_to_test = copy(self.LTM_cache['Policy'])

    def sensorial_changes(self, sensing, old_sensing):
        """Return false if all perceptions have the same value as the previous step. True otherwise."""

        self.get_logger().info('Test sensorial_changes - sensing: '+str(sensing))
        self.get_logger().info('Test sensorial_changes - old_sensing: '+str(old_sensing))


        for sensor in sensing:
            for perception, perception_old in zip(sensing[sensor], old_sensing[sensor]):
                if isinstance(perception, dict):
                    for attribute in perception:
                        difference = abs(perception[attribute] - perception_old[attribute])
                        if difference > 0.01:
                            return True
                else:
                    if abs(perception[0] - perception_old[0]) > 0.01:
                        return True
        return False
    
    def update_activations(self): # TODO(efallash): implement
        self.get_logger().info('Updating activations...')
        pass

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
    
    def get_current_reward(self): # TODO(efallash): implement
        self.get_logger().info('Reading rewards...')
        pass

    def update_pnodes_reward_basis(self): # TODO(efallash): implement
        self.get_logger().info('Updating p-nodes/c-nodes...')
        pass

    def add_point(self): # TODO(efallash): implement
        self.get_logger().info('Adding point...')
        pass

    def add_antipoint(self): # TODO(efallash): implement
        self.get_logger().info('Adding antipoint...')
        pass

    def new_cnode(self): # TODO(efallash): implement
        self.get_logger().info('Creating Cnode...')
        pass

    def reset_world(self): # TODO(efallash): implement
        self.get_logger().info('Requesting world reset...')
        pass

    def update_status(self): #TODO(efallash): implement
        self.get_logger().info('Writing files publishing status...')
        pass

    def get_current_goal(self): #TODO(efallash): implement
        self.get_logger().info('Selecting goal with higher activation...')
        pass





    def run(self): #TODO(efallash): Define if this will be a while loop or a callback-type function
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