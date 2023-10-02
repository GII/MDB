import sys
import rclpy
from rclpy.node import Node
from operator import attrgetter

from mdb.send_to_ltm_client import SendToLTMClient
from mdb.execute_policy_client import ExecutePolicyClient

class MainLoop(Node):

    def __init__(self):
        super().__init__('main_loop')
        self.iteration = 0
        self.current_policy = None
        self.run()

    def send_request_to_LTM(self, command):
        """
        Send a request to the LTM.

        :param command: The command to send.
        :type command: str
        :param name: The name of the node.
        :type name: str
        :param type: The type of the node.
        :type type: str
        :param data: Optional data.
        :type data: str
        :return: The response from the LTM.
        :rtype: mdb_interfaces.srv.SendToLTM_Response
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
        self.get_logger().info("Selecting policy...")
        # Get all policies
        # policies = []

        # Get max activation policy
        # policy = max(policies, key=attrgetter("activation"))            

        return 'policy1'

    def execute_policy(self, policy):
        self.get_logger().info('Executing policy ' + str(policy)+ '...')

        service_name = 'policy/' + str(policy) + '/execute'
        client = ExecutePolicyClient(service_name)
        policy_response = client.send_request()
        client.destroy_node()
        return policy_response

    def run(self):
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