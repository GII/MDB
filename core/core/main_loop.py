import sys
import rclpy
from rclpy.node import Node
from operator import attrgetter
import random

from core.send_to_ltm_client import SendToLTMClient
from core.execute_policy_client import ExecutePolicyClient

class MainLoop(Node):
    """
    MainLoop class for managing the main loop of the system.

    This class handles the core logic of the system, including reading perceptions,
    selecting policies, and executing policies.
    """

    def __init__(self):
        """
        Constructor for the MainLoop class.

        Initializes the MainLoop node and starts the main loop execution.
        """        
        super().__init__('main_loop')
        self.iteration = 0
        self.current_policy = None
        self.run()

    def send_request_to_LTM(self, command):
        """
        Send a request to the LTM.

        :param command: The command to send.
        :type command: str
        :return: The response from the LTM.
        :rtype: core_interfaces.srv.SendToLTM_Response
        """
        send_to_LTM_client = SendToLTMClient()
        ltm_response = send_to_LTM_client.send_request(command, '', '', '')
        send_to_LTM_client.destroy_node()
        return ltm_response

    def read_perceptions(self): # TODO: implement
        self.get_logger().info('Reading perceptions...')
        pass

    def update_activations(self): # TODO: implement
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
        client = ExecutePolicyClient(service_name)
        policy_response = client.send_request()
        client.destroy_node()
        return policy_response.policy

    def run(self):
        """
        Run the main loop of the system.
        """        
        sensing = self.read_perceptions()
        while True: # TODO: check conditions to continue the loop
            self.get_logger().info("*** ITERATION: " + str(self.iteration) + " ***")
            
            self.current_policy = self.select_policy(sensing)
            self.execute_policy(self.current_policy)
            old_sensing, sensing = sensing, self.read_perceptions()
            
            self.iteration += 1

def main(args=None):
    rclpy.init()
    main_loop = MainLoop()
    rclpy.spin(main_loop)

    main_loop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()