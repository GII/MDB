
from cognitive_nodes.drive import Drive


class DriveLineal(Drive):
    """
    Drive Lineal class
    """
    def evaluate(self, perception):
        """
        Get expected valuation for a given perception with a lineal model

        :param perception: The given normalized perception
        :type perception: dict
        :return: The valuation of the perception
        :rtype: float
        """
        return 1-perception


