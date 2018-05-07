from matplotlib import pyplot as plt
from matplotlib import patches


class StateSpace(object):
    """ Class that implements a State Space Figure 

    This figure makes possible to visualize the state spaces
    """

    def __init__(self):
        """Create the different objects needed to represent the State Spaces"""

        self.object = patches.Circle((15, 15), 3, fc=(0.8, 0, 0, 1))  # Represents the object position in SS in t
        self.object_1 = patches.Circle((15, 15), 3, fc=(0.8, 0, 0, 0.6)) #0.8 # Represents the object position in SS in t-1
        self.object_2 = patches.Circle((15, 15), 3, fc=(0.8, 0, 0, 0.2)) #0.6 # Represents the object position in SS in t-2
        # self.object_3 = patches.Circle((15, 15), 3, fc=(0.8, 0, 0, 0.4)) # 0.4 # Represents the object position in SS in t-3
        # self.object_4 = patches.Circle((15, 15), 3, fc=(0.8, 0, 0, 0.2))  # Represents the object position in SS in t-4
        # self.object_5 = patches.Circle((15, 15), 3, fc=(0.8, 0, 0, 0.0))  # Represents the object position in SS in t-5

        self.object_b = patches.Circle((215, 15), 3, fc=(0.8, 0, 0, 1))  # Represents the object position in SS in t
        self.object_1b = patches.Circle((215, 15), 3, fc=(0.8, 0, 0, 0.6))  #0.8 # Represents the object position in SS in t-1
        self.object_2b = patches.Circle((215, 15), 3, fc=(0.8, 0, 0, 0.2))  #0.6 # Represents the object position in SS in t-2
        # self.object_3b = patches.Circle((215, 15), 3, fc=(0.8, 0, 0, 0.4))  # Represents the object position in SS in t-3
        # self.object_4b = patches.Circle((215, 15), 3, fc=(0.8, 0, 0, 0.2))  # Represents the object position in SS in t-4
        # self.object_5b = patches.Circle((215, 15), 3, fc=(0.8, 0, 0, 0.0))  # Represents the object position in SS in t-5

        self.fig = plt.figure()
        #self.fig.set_size_inches((8,4))
        self.fig.canvas.set_window_title('State Space')
        self.ax = plt.axes(xlim=(0, 400), ylim=(0, 200))
        self.ax.axes.get_xaxis().set_visible(False)
        self.ax.axes.get_yaxis().set_visible(False)

        # Movement boundaries
        # Show figure and patches
        self.fig.show()
        self.ax.add_patch(self.object)
        self.ax.add_patch(self.object_1)
        self.ax.add_patch(self.object_2)
        # self.ax.add_patch(self.object_3)
        # self.ax.add_patch(self.object_4)
        # self.ax.add_patch(self.object_5)

        self.ax.add_patch(self.object_b)
        self.ax.add_patch(self.object_1b)
        self.ax.add_patch(self.object_2b)
        # self.ax.add_patch(self.object_3b)
        # self.ax.add_patch(self.object_4b)
        # self.ax.add_patch(self.object_5b)

        # State Space 1: dba1-dba2
        plt.text(170, 5, 'dBA1', fontweight='semibold')
        plt.text(12, 187, 'dBA2', fontweight='semibold')
        # plt.text(5, 15, '0.00', size='small')
        # plt.text(5, 49, '0.34', size='small')
        # plt.text(5, 83, '0.68', size='small')
        # plt.text(5, 117, '1.02', size='small')
        # plt.text(5, 151, '1.36', size='small')
        # plt.text(5, 185, '1.70', size='small')

        plt.axhline(y=15, xmin=0.0375, xmax=0.4625, linestyle='-', color='black', linewidth=1.3)
        plt.axhline(y=185, xmin=0.0375, xmax=0.4625, linestyle='-', color='black', linewidth=1.3)
        plt.axhline(y=49, xmin=0.0375, xmax=0.4625, linestyle='--', color='grey')
        plt.axhline(y=83, xmin=0.0375, xmax=0.4625, linestyle='--', color='grey')
        plt.axhline(y=117, xmin=0.0375, xmax=0.4625, linestyle='--', color='grey')
        plt.axhline(y=151, xmin=0.0375, xmax=0.4625, linestyle='--', color='grey')

        plt.text(15, 5, '0.00', size='small')
        plt.text(49, 5, '0.34', size='small')
        plt.text(83, 5, '0.68', size='small')
        plt.text(117, 5, '1.02', size='small')
        plt.text(151, 5, '1.36', size='small')
        # plt.text(185, 5, '1.70', size='small')

        plt.axvline(x=15, ymin=0.075, ymax=0.925, linestyle='-', color='black', linewidth=1.3)
        plt.axvline(x=185, ymin=0.075, ymax=0.925, linestyle='-', color='black', linewidth=1.3)
        plt.axvline(x=49, ymin=0.075, ymax=0.925, linestyle='--', color='grey')
        plt.axvline(x=83, ymin=0.075, ymax=0.925, linestyle='--', color='grey')
        plt.axvline(x=117, ymin=0.075, ymax=0.925, linestyle='--', color='grey')
        plt.axvline(x=151, ymin=0.075, ymax=0.925, linestyle='--', color='grey')

        # State Space 2: dba2-A1_reached
        plt.text(370, 5, 'dBA2', fontweight='semibold')
        plt.text(212, 187, 'A1 reached', fontweight='semibold')

        plt.text(190, 15, '0.00', size='small')
        plt.text(190, 49, '0.34', size='small')
        plt.text(190, 83, '0.68', size='small')
        plt.text(190, 117, '1.02', size='small')
        plt.text(190, 151, '1.36', size='small')
        plt.text(190, 185, '1.70', size='small')

        plt.axhline(y=15, xmin=0.5375, xmax=0.9625, linestyle='-', color='black', linewidth=1.3)
        plt.axhline(y=185, xmin=0.5375, xmax=0.9625, linestyle='-', color='black', linewidth=1.3)
        plt.axhline(y=49, xmin=0.5375, xmax=0.9625, linestyle='--', color='grey')
        plt.axhline(y=83, xmin=0.5375, xmax=0.9625, linestyle='--', color='grey')
        plt.axhline(y=117, xmin=0.5375, xmax=0.9625, linestyle='--', color='grey')
        plt.axhline(y=151, xmin=0.5375, xmax=0.9625, linestyle='--', color='grey')

        plt.text(215, 5, '0.00', size='small')
        plt.text(249, 5, '0.34', size='small')
        plt.text(283, 5, '0.68', size='small')
        plt.text(317, 5, '1.02', size='small')
        plt.text(351, 5, '1.36', size='small')
        # plt.text(385, 5, '1.70', size='small')

        plt.axvline(x=215, ymin=0.075, ymax=0.925, linestyle='-', color='black', linewidth=1.3)
        plt.axvline(x=385, ymin=0.075, ymax=0.925, linestyle='-', color='black', linewidth=1.3)
        plt.axvline(x=249, ymin=0.075, ymax=0.925, linestyle='--', color='grey')
        plt.axvline(x=283, ymin=0.075, ymax=0.925, linestyle='--', color='grey')
        plt.axvline(x=317, ymin=0.075, ymax=0.925, linestyle='--', color='grey')
        plt.axvline(x=351, ymin=0.075, ymax=0.925, linestyle='--', color='grey')

    def draw_state_space(self, sensorization):

        # x5, y5 = self.object_4.center
        # x4, y4 = self.object_3.center
        # x3, y3 = self.object_2.center
        x2, y2 = self.object_1.center
        x1, y1 = self.object.center
        x, y = sensorization[0]*100+15, sensorization[1]*100+15

        self.object.center = (x, y)
        self.object_1.center = (x1, y1)
        self.object_2.center = (x2, y2)
        # self.object_3.center = (x3, y3)
        # self.object_4.center = (x4, y4)
        # self.object_5.center = (x5, y5)

        # x5b, y5b = self.object_4b.center
        # x4b, y4b = self.object_3b.center
        # x3b, y3b = self.object_2b.center
        x2b, y2b = self.object_1b.center
        x1b, y1b = self.object_b.center
        xb, yb = sensorization[1]*100+215, sensorization[2]*100+15

        self.object_b.center = (xb, yb)
        self.object_1b.center = (x1b, y1b)
        self.object_2b.center = (x2b, y2b)
        # self.object_3b.center = (x3b, y3b)
        # self.object_4b.center = (x4b, y4b)
        # self.object_5b.center = (x5b, y5b)
        self.fig.canvas.draw()


if __name__ == '__main__':
    a = StateSpace()
