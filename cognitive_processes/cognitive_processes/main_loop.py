import sys
import rclpy
from rclpy.node import Node
from operator import attrgetter
import random

from core_interfaces.srv import SendToLTM

from core.service_client import ServiceClient
from cognitive_node_interfaces.srv import Execute


class MainLoop(Node):
    """
    MainLoop class for managing the main loop of the system.

    This class handles the core logic of the system, including reading perceptions,
    selecting policies, and executing policies.
    """

    def __init__(self, **params): #TODO(efallash): Pass parameters to this constructor (iterations, trials, options)
        """
        Constructor for the MainLoop class.

        Initializes the MainLoop node and starts the main loop execution.
        """        
        super().__init__('main_loop')
        self.iteration = 0
        self.iterations= 0
        self.trials=0
        self.current_policy = None
        self.paused=False
        self.LTM = "" #id of LTM currently being run by cognitive loop
        self.reward_threshold= 0.9

        for key, value in params.items():
            setattr(self, key, value)

        #Read LTM and configure perceptions
        self.read_ltm()
        self.configure_perceptions()


    def send_request_to_LTM(self, command): #TODO(efallash): Modify LTM service so that this method makes sense
        """
        Send a request to the LTM.

        :param command: The command to send.
        :type command: str
        :return: The response from the LTM.
        :rtype: core_interfaces.srv.SendToLTM_Response
        """
        service_name = 'send_to_LTM'
        send_to_LTM_client = ServiceClient(SendToLTM, service_name)
        ltm_response = send_to_LTM_client.send_request(command=command)
        send_to_LTM_client.destroy_node()
        return ltm_response
    
    def configure_perceptions(self): #TODO(efallash): Implement
        self.get_logger().info('')

    def read_perceptions(self): #TODO(efallash): Implement
        self.get_logger().info('Reading perceptions...')
        pass

    def receive_perception_callback(self): #TODO(efallash): Implement
        self.get_logger().info('Receiving perception')

    def read_ltm(self):
        self.get_logger().info('Reading nodes from LTM: '+ {self.LTM} + '...') # TODO(efallash): implement
        pass

    def ltm_change_callback(self):
        self.get_logger().info('Processing change from LTM...') # TODO(efallash): implement
        pass


    def select_policy(self, sensing): # TODO: implement
        """
        Select a policy based on the current sensing.

        Mock method that selects a random policy between policy1 and policy2.

        :param sensing: The current sensing.
        :type sensing: Any
        :return: The selected policy.
        :rtype: str
        """
        self.get_logger().info("Selecting policy...")
        id = random.randint(1,2)
        
        # Get all policies
        # policies = []

        # Get max activation policy
        # policy = max(policies, key=attrgetter("activation"))            

        return 'policy' + str(id)
    
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

    def update_policies_to_test(self): # TODO(efallash): implement
        self.get_logger().info('Updating test policies list...')
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

        self.get_logger().info("Running MDB with LTM:", str(self.LTM))

        sensing = self.read_perceptions()
        stm=[]
        while (self.iteration<=self.iterations) and (not self.paused): # TODO: check conditions to continue the loop


            self.get_logger().info("*** ITERATION: " + str(self.iteration) + " ***")
            
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

            self.update_status()
            self.iteration += 1

    




        

def main(args=None):

    main_loop = MainLoop()

    main_loop.get_logger().info('Runnning node')
    main_loop.run()

    main_loop.get_logger().info('Spinning node')
    rclpy.spin(main_loop) #TODO(efallash): Does not make much sense to spin the node as the node will be stuck in the while loop.
                          #Check callback implications of the while loop
                          #https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html
                          #https://discourse.ros.org/t/how-to-use-callback-groups-in-ros2/25255
                          #https://nicolovaligi.com/articles/concurrency-and-parallelism-in-ros1-and-ros2-application-apis/ Rospy.spin (ros1) does not interfere with callbacks 
    

    main_loop.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()