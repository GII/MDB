import numpy as np


class ActionChooser(object):
    """Class that implements an Action  Chooser.
    
    This Action Chooser is in charge of creating the candidate actions to evaluate in the Candidate State Evaluator.
    At the same time, it is fed by the candidate actions ordered according to a valuation given by the Candidate
    State Evaluator and chooses the one it considers the best.
    """

    def __init__(self):
        self.l_sup = 60
        self.l_inf = -60

    def getCandidateActions(self, n=25):
        """Return a list of candidate actions to apply in the robots.
        
        :param n: int that indicates the number of candidate actions generated (default 20 actions)
        :return: candidate_actions: a list of tuples of candidate actions. Each tuple contains 3 different angles (one for each robot)
        """
        candidate_actions = []
        for i in range(n):
            robobo_angle = np.random.uniform(self.l_sup, self.l_inf)
            baxter_l_angle = np.random.uniform(self.l_sup, self.l_inf)

            candidate_actions.append((robobo_angle, baxter_l_angle))

        return candidate_actions

    def chooseAction(self, candidate_actions):
        """Return the action selected according to some criterion.
        
        :param candidate_actions: List of tuples with actions ordered according to a valuation 
        (act_robobo, act_baxter_l, act_baxter_r, valuation)
        :return: action: Tuple with the action (3 angles) chosen to apply
        """

        # In a first approximation, the action chosen is the one with the best valuation, the last of the list
        action = list(candidate_actions[-1])  # Convert into a list to remove the valuation value

        return action[0]
