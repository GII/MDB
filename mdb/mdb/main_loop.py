import sys
import rclpy
from rclpy.node import Node

from mdb.send_to_ltm_client import SendToLTMClient

class MainLoop(Node):

    def __init__(self):
        super().__init__('main_loop')
        self.iteration = 0
        self.current_policy = None

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
        pass

    def update_activations(self): # TODO: implement
        pass

    def select_policy(self, sensing):
        self.update_activations(sensing, new_sensings=True)
        # 1
        policies = [] # TODO: get policies from LTM
        # policy = max(policies, key=attrgetter("activation"))
        # 2
        # TODO: #3 #4 #5
        return policy


    def run(self):
        sensing = self.read_perceptions()
        while True: # TODO: check conditions to continue the loop
            self.get_logger().info("*** ITERATION: " + str(self.iteration) + " ***")
            
            self.current_policy = self.select_policy(sensing)
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