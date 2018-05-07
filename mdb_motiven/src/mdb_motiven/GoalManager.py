from Goal import *

class GoalManager(object):
    """Class that represents the Goal Manager
    
    This component includes different processes involved in achieving the goal,
    mainly the sub-goal identification and combination. It provides the Candidate
    State Evaluator with some criteria to evaluate the sensorial candidate states,
    and it requires the current set of motivations from the Motivation Manager in
    order to create the sub-goals.
    
    """

    def __init__(self):
        self.goals = []

    def newGoal(self, id):
        """This method is used to add a new goal to the list when it is discovered"""
        self.goals.append(Goal(id))

    def getGoalsList(self):
        """This method is used to obtain the list with all the goals the system has"""
        return self.goals


    # def goalAchieved(self, reward):
    #     if reward:
    #         return 1
    #     else:
    #         return 0
