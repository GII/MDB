
from cognitive_nodes.drive import Drive


class DriveLineal(Drive):

    def evaluate(self, perception):
        return 1-perception


