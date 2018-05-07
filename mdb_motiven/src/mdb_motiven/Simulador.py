import math
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import patches


# Para el primer ejemplo debo cambiar cosas o tener en cuenta que, por ejemplo, en el vector de sensorizacion solo
# debo considerar la distancia del baxter a la pelota y de la pelota a la caja

class Sim(object):
    """ Class that implements a Simulator
    
    This simulator makes possible to make/test different experiments in a virtual scenario.
    It contains the two arms of the Baxter robot, the Robobo! robot, some boxes and a
    ball.
    
    The implemented methods allow the user to move both robots throw the scenario (with
    and without the ball), get distances and relative angles between the different objects 
    and get/set the position of all of them.
    """

    def __init__(self):
        """Create the different objects present in the Simulator and place them in it"""
        self.ball = plt.Circle((1800, 300), 30, fc='r', label='ball')
        self.box1 = plt.Rectangle((1800, 450), 67, 67, fc='y', linewidth=3.5, label='box1')
        self.box2 = plt.Rectangle((2000, 900), 67, 67, fc='grey', linewidth=3.5, label='box2')
        self.robobo = patches.Rectangle((700, 300), 75, 60, angle=0.0, fc='cyan', label='robobo')
        self.robobo_act = patches.Rectangle((775, 300), 20, 60, angle=0.0, fc='blue', label='robobo_actuator')
        self.baxter_rarm = patches.Rectangle((2000, 50), 75, 60, angle=0.0, fc=(0.8, 0, 0.2), label='baxter_rarm')
        self.baxter_rarm_act = patches.Rectangle((2075, 50), 20, 60, angle=0.0, fc='black', label='baxter_rarm_act')
        self.baxter_larm = patches.Rectangle((1600, 50), 75, 60, angle=0.0, fc=(0.8, 0, 0.2), label='baxter_larm')
        self.baxter_larm_act = patches.Rectangle((1675, 50), 20, 60, angle=0.0, fc='black', label='baxter_larm_act')

        self.baxter_figure = patches.Circle((2700, 264), 12, fc=(0.8, 0, 0, 1))
        self.baxter_figure_1 = patches.Circle((2700, 264), 12, fc=(0.8, 0, 0, 0.8))
        self.baxter_figure_2 = patches.Circle((2700, 264), 12, fc=(0.8, 0, 0, 0.6))
        self.baxter_figure_3 = patches.Circle((2700, 264), 12, fc=(0.8, 0, 0, 0.4))
        self.baxter_figure_4 = patches.Circle((2700, 264), 12, fc=(0.8, 0, 0, 0.2))
        self.baxter_figure_5 = patches.Circle((2700, 264), 12, fc=(0.8, 0, 0, 0.0))

        self.fig = plt.figure()
        self.fig.canvas.set_window_title('Simulator')
        self.ax = plt.axes(xlim=(0, 3500), ylim=(0, 1000))
        self.ax.axes.get_xaxis().set_visible(False)
        self.ax.axes.get_yaxis().set_visible(False)

        # Movement boundaries
        plt.axvline(x=1250)  # draw a default vline at x=1 that spans the yrange
        plt.axhline(y=800, xmin=0.357, xmax=0.686, linestyle='--', color='grey')
        plt.axhline(y=50, xmin=0.0286, xmax=0.686, linestyle='--', color='grey')
        # plt.axhline(y=950, xmin=0.0286, xmax=0.357, linestyle='--', color='grey')
        plt.axhline(y=800, xmin=0.0286, xmax=0.357, linestyle='--', color='grey')
        # plt.axvline(x=100, ymin=0.05, ymax=0.95, linestyle='--', color='grey')
        plt.axvline(x=100, ymin=0.05, ymax=0.80, linestyle='--', color='grey')
        plt.axvline(x=2400, ymin=0.05, ymax=0.80, linestyle='--', color='grey')
        self.ball_position = None  # Indicates where is the ball: robobo, baxter_larm, bater_rarm, box1, box2 or None

        # Show figure and patches
        self.fig.show()
        self.ax.add_patch(self.box1)
        self.ax.add_patch(self.box2)
        self.ax.add_patch(self.robobo)
        self.ax.add_patch(self.robobo_act)
        # self.ax.add_patch(self.baxter_rarm)
        # self.ax.add_patch(self.baxter_rarm_act)
        self.ax.add_patch(self.baxter_larm)
        self.ax.add_patch(self.baxter_larm_act)
        self.ax.add_patch(self.ball)

        #plt.text(2700, 970, 'State Space')

        # Prueba espacio estados
        plt.axhline(y=950, xmin=0.771, xmax=0.967, linestyle='-', color='black', linewidth=1.3)
        plt.axhline(y=264, xmin=0.771, xmax=0.967, linestyle='-', color='black', linewidth=1.3)
        plt.axhline(y=364, xmin=0.771, xmax=0.967, linestyle='--', color='grey')
        plt.axhline(y=464, xmin=0.771, xmax=0.967, linestyle='--', color='grey')
        plt.axhline(y=564, xmin=0.771, xmax=0.967, linestyle='--', color='grey')
        plt.axhline(y=664, xmin=0.771, xmax=0.967, linestyle='--', color='grey')
        plt.axhline(y=764, xmin=0.771, xmax=0.967, linestyle='--', color='grey')
        plt.axhline(y=864, xmin=0.771, xmax=0.967, linestyle='--', color='grey')
        plt.axvline(x=2700, ymin=0.264, ymax=0.950, linestyle='-', color='black', linewidth=1.3)
        plt.axvline(x=3386, ymin=0.264, ymax=0.950, linestyle='-', color='black', linewidth=1.3)
        plt.axvline(x=2800, ymin=0.264, ymax=0.950, linestyle='--', color='grey')
        plt.axvline(x=2900, ymin=0.264, ymax=0.950, linestyle='--', color='grey')
        plt.axvline(x=3000, ymin=0.264, ymax=0.950, linestyle='--', color='grey')
        plt.axvline(x=3100, ymin=0.264, ymax=0.950, linestyle='--', color='grey')
        plt.axvline(x=3200, ymin=0.264, ymax=0.950, linestyle='--', color='grey')
        plt.axvline(x=3300, ymin=0.264, ymax=0.950, linestyle='--', color='grey')
        plt.axvline(x=2500)
        self.ax.add_patch(self.baxter_figure)
        self.ax.add_patch(self.baxter_figure_1)
        self.ax.add_patch(self.baxter_figure_2)
        self.ax.add_patch(self.baxter_figure_3)
        self.ax.add_patch(self.baxter_figure_4)
        self.ax.add_patch(self.baxter_figure_5)

    def ball_set_pos(self, (x, y)):
        """Set the ball center position"""
        self.ball.center = (x, y)
        # self.fig.canvas.draw()

    def box1_set_pos(self, (x, y)):
        """Set the box1 center position"""
        self.box1.xy = (x - self.box1.get_width() / 2, y - self.box1.get_height() / 2)
        # self.fig.canvas.draw()

    def box2_set_pos(self, (x, y)):
        """Set the box2 center position"""
        self.box2.xy = (x - self.box2.get_width() / 2, y - self.box2.get_height() / 2)
        # self.fig.canvas.draw()

    def robobo_set_pos(self, (x, y)):
        """Set the Robobo! center position (and its actuator) checking if it is inside its movement limits"""
        w = self.robobo.get_width()
        h = self.robobo.get_height()
        # New x, y position
        new_x = x - w / 2 * math.cos(self.robobo._angle * math.pi / 180) + h / 2 * math.sin(
            self.robobo._angle * math.pi / 180)
        new_y = y - w / 2 * math.sin(self.robobo._angle * math.pi / 180) - h / 2 * math.cos(
            self.robobo._angle * math.pi / 180)

        # Check if the new robobo position is inside its movement limits
        if self.check_limits((x, y), 'robobo'):
            self.robobo.xy = (new_x, new_y)
            self.robobo_act.xy = (new_x + w * math.cos(self.robobo_act._angle * math.pi / 180),
                                  new_y + w * math.sin(self.robobo_act._angle * math.pi / 180))
        # self.fig.canvas.draw()

    def robobo_set_angle(self, angle):
        """Set the Robobo! angle (and its actuator angle and adjust its position)"""
        centre = self.robobo_get_pos()
        self.robobo._angle = angle
        self.robobo_act._angle = angle
        self.robobo_set_pos(centre)
        w = self.robobo.get_width()
        self.robobo_act.xy = (self.robobo.get_x() + w * math.cos(self.robobo_act._angle * math.pi / 180),
                              self.robobo.get_y() + w * math.sin(self.robobo_act._angle * math.pi / 180))
        if self.ball_position == 'robobo':
            self.ball_set_pos(self.robobo_act_get_pos())
        # self.fig.canvas.draw()

    # def baxter_rarm_set_pos(self, (x, y)):
    #     """Set the Baxter's right arm center position (and its actuator) checking if it is inside its movement limits"""
    #     w = self.baxter_rarm.get_width()
    #     h = self.baxter_rarm.get_height()
    #     # New x, y position
    #     new_x = x - w / 2 * math.cos(self.baxter_rarm._angle * math.pi / 180) + h / 2 * math.sin(
    #         self.baxter_rarm._angle * math.pi / 180)
    #     new_y = y - w / 2 * math.sin(self.baxter_rarm._angle * math.pi / 180) - h / 2 * math.cos(
    #         self.baxter_rarm._angle * math.pi / 180)
    #
    #     # Check if the new baxter right arm position is inside its movement limits
    #     if self.check_limits((x, y), 'baxter'):
    #         self.baxter_rarm.xy = (new_x, new_y)
    #         self.baxter_rarm_act.xy = (new_x + w * math.cos(self.baxter_rarm_act._angle * math.pi / 180),
    #                                    new_y + w * math.sin(self.baxter_rarm_act._angle * math.pi / 180))
    #     self.fig.canvas.draw()
    #
    # def baxter_rarm_set_angle(self, angle):
    #     """Set the Baxter's right arm angle (and its actuator angle and adjust its position)"""
    #     centre = self.baxter_rarm_get_pos()
    #     self.baxter_rarm._angle = angle
    #     self.baxter_rarm_act._angle = angle
    #     self.baxter_rarm_set_pos(centre)
    #     w = self.baxter_rarm.get_width()
    #     self.baxter_rarm_act.xy = (self.baxter_rarm.get_x() + w * math.cos(self.baxter_rarm_act._angle * math.pi / 180),
    #                                self.baxter_rarm.get_y() + w * math.sin(self.baxter_rarm_act._angle * math.pi / 180))
    #     if self.ball_position == 'baxter_rarm':
    #         self.ball_set_pos(self.baxter_rarm_act_get_pos())
    #     self.fig.canvas.draw()

    ## def baxter_rarm_set_height(self, h):
    ##    self.baxter_rarm._angle = angle
    ##    self.fig.canvas.draw()

    def baxter_larm_set_pos(self, (x, y)):
        """Set the Baxter's left arm center position (and its actuator) checking if it is inside its movement limits"""
        w = self.baxter_larm.get_width()
        h = self.baxter_larm.get_height()
        # New x, y position
        new_x = x - w / 2 * math.cos(self.baxter_larm._angle * math.pi / 180) + h / 2 * math.sin(
            self.baxter_larm._angle * math.pi / 180)
        new_y = y - w / 2 * math.sin(self.baxter_larm._angle * math.pi / 180) - h / 2 * math.cos(
            self.baxter_larm._angle * math.pi / 180)

        # Check if the new baxter left arm position is inside its movement limits
        if self.check_limits((x, y), 'baxter'):
            self.baxter_larm.xy = (new_x, new_y)
            self.baxter_larm_act.xy = (new_x + w * math.cos(self.baxter_larm_act._angle * math.pi / 180),
                                       new_y + w * math.sin(self.baxter_larm_act._angle * math.pi / 180))
        # self.fig.canvas.draw()

    def baxter_larm_set_angle(self, angle):
        """Set the Baxter's left arm angle (and its actuator angle and adjust its position)"""
        centre = self.baxter_larm_get_pos()
        self.baxter_larm._angle = angle
        self.baxter_larm_act._angle = angle
        self.baxter_larm_set_pos(centre)
        w = self.baxter_larm.get_width()
        self.baxter_larm_act.xy = (self.baxter_larm.get_x() + w * math.cos(self.baxter_larm_act._angle * math.pi / 180),
                                   self.baxter_larm.get_y() + w * math.sin(self.baxter_larm_act._angle * math.pi / 180))
        if self.ball_position == 'baxter_larm':
            self.ball_set_pos(self.baxter_larm_act_get_pos())
        # self.fig.canvas.draw()

    def ball_get_pos(self):
        """Return the position of the center of the ball"""
        return self.ball.center

    def box1_get_pos(self):
        """Return the position of the center of the box1"""
        return tuple(map(sum, zip(self.box1.xy, (self.box1.get_width() / 2, self.box1.get_height() / 2))))

    def box2_get_pos(self):
        """Return the position of the center of the box2"""
        return tuple(map(sum, zip(self.box2.xy, (self.box2.get_width() / 2, self.box2.get_height() / 2))))

    def robobo_get_pos(self):
        """Return the position of the center of the Robobo!"""
        w = self.robobo.get_width()
        h = self.robobo.get_height()
        x, y = self.robobo.xy
        x_c = x + w / 2 * math.cos(self.robobo._angle * math.pi / 180) - h / 2 * math.sin(
            self.robobo._angle * math.pi / 180)
        y_c = y + w / 2 * math.sin(self.robobo._angle * math.pi / 180) + h / 2 * math.cos(
            self.robobo._angle * math.pi / 180)
        return x_c, y_c

    def robobo_act_get_pos(self):
        """Return the position of the center of the Robobo! actuator"""
        w = self.robobo_act.get_width()
        h = self.robobo_act.get_height()
        x, y = self.robobo_act.xy
        x_c = x + w / 2 * math.cos(self.robobo_act._angle * math.pi / 180) - h / 2 * math.sin(
            self.robobo_act._angle * math.pi / 180)
        y_c = y + w / 2 * math.sin(self.robobo_act._angle * math.pi / 180) + h / 2 * math.cos(
            self.robobo_act._angle * math.pi / 180)
        return x_c, y_c

    def robobo_get_angle(self):
        """Return the angle of the Robobo!"""
        return self.robobo._angle
    #
    # def baxter_rarm_get_pos(self):
    #     """Return the position of the center of the Baxter's right arm"""
    #     w = self.baxter_rarm.get_width()
    #     h = self.baxter_rarm.get_height()
    #     x, y = self.baxter_rarm.xy
    #     x_c = x + w / 2 * math.cos(self.baxter_rarm._angle * math.pi / 180) - h / 2 * math.sin(
    #         self.baxter_rarm._angle * math.pi / 180)
    #     y_c = y + w / 2 * math.sin(self.baxter_rarm._angle * math.pi / 180) + h / 2 * math.cos(
    #         self.baxter_rarm._angle * math.pi / 180)
    #     return x_c, y_c
    #
    # def baxter_rarm_act_get_pos(self):
    #     """Return the position of the center of the Baxter's right arm actuator"""
    #     w = self.baxter_rarm_act.get_width()
    #     h = self.baxter_rarm_act.get_height()
    #     x, y = self.baxter_rarm_act.xy
    #     x_c = x + w / 2 * math.cos(self.baxter_rarm_act._angle * math.pi / 180) - h / 2 * math.sin(
    #         self.baxter_rarm_act._angle * math.pi / 180)
    #     y_c = y + w / 2 * math.sin(self.baxter_rarm_act._angle * math.pi / 180) + h / 2 * math.cos(
    #         self.baxter_rarm_act._angle * math.pi / 180)
    #     return x_c, y_c

    def baxter_larm_get_pos(self):
        """Return the position of the center of the Baxter's left arm"""
        w = self.baxter_larm.get_width()
        h = self.baxter_larm.get_height()
        x, y = self.baxter_larm.xy
        x_c = x + w / 2 * math.cos(self.baxter_larm._angle * math.pi / 180) - h / 2 * math.sin(
            self.baxter_larm._angle * math.pi / 180)
        y_c = y + w / 2 * math.sin(self.baxter_larm._angle * math.pi / 180) + h / 2 * math.cos(
            self.baxter_larm._angle * math.pi / 180)
        return x_c, y_c

    def baxter_larm_act_get_pos(self):
        """Return the position of the center of the Baxter's left arm actuator"""
        w = self.baxter_larm_act.get_width()
        h = self.baxter_larm_act.get_height()
        x, y = self.baxter_larm_act.xy
        x_c = x + w / 2 * math.cos(self.baxter_larm_act._angle * math.pi / 180) - h / 2 * math.sin(
            self.baxter_larm_act._angle * math.pi / 180)
        y_c = y + w / 2 * math.sin(self.baxter_larm_act._angle * math.pi / 180) + h / 2 * math.cos(
            self.baxter_larm_act._angle * math.pi / 180)
        return x_c, y_c

    # def baxter_rarm_get_angle(self):
    #     """Return the angle of the right arm of the Baxter robot"""
    #     return self.baxter_rarm._angle

    def baxter_larm_get_angle(self):
        """Return the angle of the left arm of the Baxter robot"""
        return self.baxter_larm._angle

    def get_distance(self, (x1, y1), (x2, y2)):
        """Return the distance between two points"""
        return math.sqrt(pow(x2 - x1, 2) + (pow(y2 - y1, 2)))

    def get_relative_angle(self, (x1, y1), (x2, y2)):
        """Return the relative angle betwen two points"""
        return math.atan2(y2 - y1, x2 - x1) * 180 / math.pi

    # def move_baxter_rarm(self, vel=10):
    #     """Move Baxter's right arm wih a specific velocity (default 10)"""
    #     x, y = self.baxter_rarm_get_pos()
    #     self.baxter_rarm_set_pos((x + vel * math.cos(self.baxter_rarm._angle * math.pi / 180),
    #                               y + vel * math.sin(self.baxter_rarm._angle * math.pi / 180)))
    #     if self.ball_position == 'baxter_rarm':
    #         self.ball_set_pos(self.baxter_rarm_act_get_pos())

    def move_baxter_larm(self, vel=20):
        """Move Baxter's left arm wih a specific velocity (default 20)"""
        x, y = self.baxter_larm_get_pos()
        self.baxter_larm_set_pos((x + vel * math.cos(self.baxter_larm._angle * math.pi / 180),
                                  y + vel * math.sin(self.baxter_larm._angle * math.pi / 180)))
        if self.ball_position == 'baxter_larm':
            self.ball_set_pos(self.baxter_larm_act_get_pos())

    def baxter_larm_action(self, relative_angle, vel=20):
        """Move baxter left arm with a specific angle and velocity (default 20)"""
        angle = self.baxter_larm._angle + relative_angle
        self.baxter_larm_set_angle(angle)
        self.move_baxter_larm(vel)
        self.world_rules()
        # self.baxter_state_space()

    def move_robobo(self, vel=20):
        """Move Robobo! wih a specific velocity (default 20)"""
        x, y = self.robobo_get_pos()
        self.robobo_set_pos((x + vel * math.cos(self.robobo._angle * math.pi / 180),
                             y + vel * math.sin(self.robobo._angle * math.pi / 180)))
        if self.ball_position == 'robobo':
            self.ball_set_pos(self.robobo_act_get_pos())

    def robobo_action(self, relative_angle, vel=20):
        """Move robobo with a specific angle and velocity (default 20)"""
        angle = self.robobo._angle + relative_angle
        self.robobo_set_angle(angle)
        self.move_robobo(vel)
        self.world_rules()
        # self.baxter_state_space()

    def apply_action(self, relative_angles, vel_rob=25, vel_baxt=25):
        """Move robobo and baxter left arm with a specific angle and velocity (default 20)"""
        self.robobo_action(relative_angles[0], vel_rob)
        self.baxter_larm_action(relative_angles[1], vel_baxt)


    # def move_baxter_rarm_ball(self, vel=10):
    #     x, y = self.baxter_rarm_get_pos()
    #     self.baxter_rarm_set_pos((x + vel * math.cos(self.baxter_rarm._angle * math.pi / 180),
    #                               y + vel * math.sin(self.baxter_rarm._angle * math.pi / 180)))
    #     self.ball_set_pos(self.baxter_rarm_act_get_pos())
    #
    # def move_baxter_larm_ball(self, vel=10):
    #     x, y = self.baxter_larm_get_pos()
    #     self.baxter_larm_set_pos((x + vel * math.cos(self.baxter_larm._angle * math.pi / 180),
    #                               y + vel * math.sin(self.baxter_larm._angle * math.pi / 180)))
    #     self.ball_set_pos(self.baxter_larm_act_get_pos())
    #
    # def move_robobo_ball(self, vel=10):
    #     x, y = self.robobo_get_pos()
    #     self.robobo_set_pos((x + vel * math.cos(self.robobo._angle * math.pi / 180),
    #                          y + vel * math.sin(self.robobo._angle * math.pi / 180)))
    #     self.ball_set_pos(self.robobo_act_get_pos())

    def get_reward(self):
        """Return the reward checking if the ball is inside one of the boxes"""
        if self.ball_position == 'box1':  # or self.ball_position == 'box2':
            reward = 1
        else:
            reward = 0

        return reward

    def get_sensorization(self):
        """Return a sensorization vector with the distances between the ball and the robots' actuators and the reward"""
        dist_rob_ball = self.get_distance(self.robobo_act_get_pos(), self.ball_get_pos())
        dist_baxt_larm_ball = self.get_distance(self.baxter_larm_act_get_pos(), self.ball_get_pos())
        # dist_baxt_rarm_ball = self.get_distance(self.baxter_rarm_act_get_pos(), self.ball_get_pos())
        dist_ball_box1 = self.get_distance(self.ball_get_pos(), self.box1_get_pos())
        # dist_ball_box2 = self.get_distance(self.ball_get_pos(), self.box2_get_pos())
        # reward = self.get_reward()

        return dist_rob_ball, dist_baxt_larm_ball, dist_ball_box1  # ,reward

    def world_rules(self):
        """Establish the ball position in the scenario"""
        # Set minimum distances for the ball to be inside the box or grasped by a robot
        min_dist_box = 50
        min_dist_robot = 50
        # Ball position: Check where the ball has to be (check distances)
        sens = self.get_sensorization()
        if sens[2] < min_dist_box and self.ball_position == 'baxter_larm':  # distance ball-box1
            # if self.ball_position == 'baxter_larm':
            #     self.baxter_larm_set_pos(self.box1_get_pos())
            self.ball_position = 'box1'
            # self.ball_set_pos(self.box1_get_pos())
        elif sens[1] < min_dist_robot:  # distance ball-baxter_larm
            self.ball_position = 'baxter_larm'
            self.ball_set_pos(self.baxter_larm_act_get_pos())
        elif sens[0] < min_dist_robot:  # distance ball-robobo
            self.ball_position = 'robobo'
            self.ball_set_pos(self.robobo_act_get_pos())
        else:
            self.ball_position = None

    def check_limits(self, (x, y), robot_type):
        """Check if the next position of one of the robots is inside its movement limits"""
        # Set limits for robots movements
        lim_robobo_x = (100, 2400)#(100, 1250)  # (x_min,x_max)
        lim_robobo_y = (50, 800)#(50, 950)
        lim_baxter_x = (1250, 2400)
        lim_baxter_y = (50, 800)
        # Movement boundaries
        result = 1
        if robot_type == 'robobo':
            if x < lim_robobo_x[0] or x > lim_robobo_x[1] or y < lim_robobo_y[0] or y > lim_robobo_y[1]:
                result = 0  # Do not move: it may crash
        elif robot_type == 'baxter':
            if x < lim_baxter_x[0] or x > lim_baxter_x[1] or y < lim_baxter_y[0] or y > lim_baxter_y[1]:
                result = 0  # Do not move: exceed movement boundaries

        return result

    def restart_scenario(self):
        self.ball_set_pos((np.random.randint(100, 2400), np.random.randint(50, 800)))
        # self.box1_set_pos((np.random.randint(1750, 2400), np.random.randint(50, 800)))
        # self.box1_set_pos((np.random.randint(1250, 2400), np.random.randint(50, 800)))

    # def baxter_state_space(self):

        # x5, y5 = self.baxter_figure_4.center
        # x4, y4 = self.baxter_figure_3.center
        # x3, y3 = self.baxter_figure_2.center
        # x2, y2 = self.baxter_figure_1.center
        # x1, y1 = self.baxter_figure.center
        #
        # x, y = 2700 + self.get_sensorization()[0] / 2, 264 + self.get_sensorization()[1] / 2
        #
        # self.baxter_figure.center = (x, y)
        # self.baxter_figure_1.center = (x1, y1)
        # self.baxter_figure_2.center = (x2, y2)
        # self.baxter_figure_3.center = (x3, y3)
        # self.baxter_figure_4.center = (x4, y4)
        # self.baxter_figure_5.center = (x5, y5)
        # self.fig.canvas.draw()

if __name__ == '__main__':
    a = Sim()
