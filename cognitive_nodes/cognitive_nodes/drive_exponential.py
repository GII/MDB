
from cognitive_nodes.drive import Drive


class DriveExponential(Drive):
    """
    Drive Exponential class
    """
    def evaluate(self, perception):
        """
        Get expected valuation for a given perception with an exponential model

        :param perception: The given normalized perception
        :type perception: dict
        :return: The valuation of the perception
        :rtype: float
        """
        return 100 * 0.8 ** (perception * 10.0)


