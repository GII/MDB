
from cognitive_nodes.drive import Drive


class DriveExponential(Drive):

    def evaluate(self, perception):
        return 100 * 0.8 ** (perception * 10.0)


