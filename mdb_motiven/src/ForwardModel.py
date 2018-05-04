import math


class ForwardModel(object):
    """Class tha represents a Forward Model, a model of the World.
    
    It contains the methods needed to predict the next State Space created with the application of a particular
    action in the actual State Space.
    """

    def predictedState(self, candidate_action, (bax_l_pos, bax_l_angle, rob_pos, rob_angle, ball_pos, ball_situation, box1_pos, desp_vel)):
        """Return the predicted sensorization vector calculated as a function of the actual states and the particular 
        actions applied to the robots.
        
        :param bax_l_action: Tuple containing the action applied (vel, angle) to the left arm of the Baxter robot
        :param bax_l_pos: Tuple containing the position (x, y) of the center of the left arm of the Baxter robot
        :param bax_l_angle: actual Angle of the baxter actuator
        :param rob_pos: Tuple containing the position (x, y) of the center of the Robobo robot
        :param rob_angle: actual Angle of the Robobo actuator
        :param ball_pos: Tuple containing the position (x, y) of the center of the ball
        :param ball_situation: String containing the situation of the ball in the scenario ('robobo','baxter_rarm', 'bxter_larm' or else)
        :param box1_pos: Tuple containing the position (x, y) of the center of the box 1
        :param w: width of the rectangle that represents the robot
        :param w_act: width of the rectangle that represents the robot actuator
        :param h: height of the rectangle that represents the robot and the rectangle that represents the robot actuator
        :return: sens:Tuple containing the distances between the ball and the robots' actuators and the reward
        """

        # Predicted Baxter left arm position after applying the particular action
        rob_act_new_pos  = self.get_act_new_pos(rob_pos, rob_angle, candidate_action, 'robobo', desp_vel)
        # Predicted Baxter left arm position after applying the particular action
        bax_l_act_new_pos = self.get_act_new_pos(bax_l_pos, bax_l_angle, candidate_action, 'baxter', desp_vel)
        # Predicted ball position attending to its former situation
        ball_new_pos = self.get_ball_pos(ball_pos, ball_situation, rob_act_new_pos, bax_l_act_new_pos)

        sens = self.get_sensorization(rob_act_new_pos, bax_l_act_new_pos, ball_new_pos, box1_pos)
        return sens

    def get_ball_pos(self, ball_pos, situation, rob_pos, bax_l_pos):
        """Returns the predicted position of the ball according to its current situation.
    
        :param ball_pos: Tuple containing the position (x, y) of the center of the ball
        :param situation: String that indicates the situation of the ball in the scenario ('robobo','baxter_rarm', 'bxter_larm' or else)
        :param bax_l_pos: Tuple containing the new position (x, y) of the center of the left arm of the Baxter robot
        :return: Tuple containing the new position (x, y) of the center of the ball
        """

        if situation[1] == 0: #'baxter_larm':
            ball_new_pos = bax_l_pos
        elif situation[0] == 0: #'robobo':
            ball_new_pos = rob_pos
        else:
            ball_new_pos = ball_pos

        return ball_new_pos

    def get_sensorization(self, rob_act_pos, baxter_l_act_pos, ball_pos, box1_pos):
        """Return a sensorization vector with the distances between the ball and the robots' actuators.
        
        :param rob_act_pos: Tuple containing the new position (x, y) of the center of the actuator of the Robobo robot
        :param baxter_l_act_pos: Tuple containing the new position (x, y) of the center of the actuator of the left arm of the Baxter robot
        :param ball_pos: Tuple containing the new position (x, y) of the center of the ball
        :param box1_pos: Tuple containing the position (x, y) of the center of the box 1
        :return: Tuple containing the distances between the ball and the robots' actuators
        """
        dist_rob_ball = self.get_distance(rob_act_pos, ball_pos)
        dist_baxt_larm_ball = self.get_distance(baxter_l_act_pos, ball_pos)
        dist_ball_box1 = self.get_distance(ball_pos, box1_pos)

        return dist_rob_ball, dist_baxt_larm_ball, dist_ball_box1

    def get_distance(self, (x1, y1), (x2, y2)):
        """Return the distance between two points (x1, y1) and (x2, y2)"""
        return math.sqrt(pow(x2 - x1, 2) + (pow(y2 - y1, 2)))

    def get_act_new_pos(self, (x, y), angle, rel_angle, robot, vel=0.05*1000):
        """Returns the new position of the actuator of the robot.
        
        :param x, y: Tuple containing the position (x, y) of the center of the robot
        :param angle: actual angle of the robot
        :param rel_angle: candidate relative angle to apply to the robot
        :param w:  width of the rectangle that represents the robot
        :param h: height of the rectangle that represents the robot 
        :param w_act:  width of the rectangle that represents the robot actuator
        :param h_act: height of the rectangle that represents the robot actuator
        :param vel: velocity of movement (default = 10)
        :return: Tuple containing the new position (x, y) of the center of the robot actuator
        """
        # angle += rel_angle
        #
        # x_new, y_new = self.get_new_pos((x, y), vel, angle, w, h)
        #
        # x_act, y_act = (x_new + w * math.cos(angle * math.pi / 180), y_new + w * math.sin(angle * math.pi / 180))
        #
        # xc_act = x_act + w_act / 2 * math.cos(angle * math.pi / 180) - h_act / 2 * math.sin(angle * math.pi / 180)
        # yc_act = y_act + w_act / 2 * math.sin(angle * math.pi / 180) + h_act / 2 * math.cos(angle * math.pi / 180)
        
        if robot == 'baxter':
            if rel_angle.baxter_valid.data == False:
                x_act = x
                y_act = y
            else:
                angle += rel_angle.baxter_action.data
                # New center position
                x_act = x + vel * math.cos(angle * math.pi / 180.0)
                y_act = y + vel * math.sin(angle * math.pi / 180.0)
        elif robot == 'robobo':
            if rel_angle.robobo_valid.data == False:
                x_act = x + 100 * math.cos(angle * math.pi / 180.0)
                y_act = y + 100 * math.sin(angle * math.pi / 180.0)
            else:
                angle += rel_angle.robobo_action.data
                # New center position
                x_act = x + vel * math.cos(angle * math.pi / 180.0) + 100 * math.cos(angle * math.pi / 180.0) # Proy_x + pos_grip_x
                y_act = y + vel * math.sin(angle * math.pi / 180.0) + 100 * math.sin(angle * math.pi / 180.0) # Proy_y + pos_grip_y

        return x_act, y_act

    # def get_new_pos(self, (x, y), vel, angle, w, h):
    #     """Returns the new position of the robot after applying a particular action defined by (vel, angle).
    #
    #     :param x, y: Tuple containing the position (x, y) of the center of the robot
    #     :param vel: A value that represents the velocity of motion
    #     :param angle: A value that represents the orientation choosen to move forward (0-360)
    #     :param w:  width of the rectangle that represents the robot
    #     :param h: height of the rectangle that represents the robot
    #     :return: A tuple containing the position (x, y) of the left lower vertex of the robot after applying a particular action
    #     """
    #
    #     # New center position
    #     new_xc2 = x + vel * math.cos(angle * math.pi / 180)
    #     new_yc2 = y + vel * math.sin(angle * math.pi / 180)
    #     # New x, y position
    #     new_x2 = new_xc2 - w / 2 * math.cos(angle * math.pi / 180) + h / 2 * math.sin(angle * math.pi / 180)
    #     new_y2 = new_yc2 - w / 2 * math.sin(angle * math.pi / 180) - h / 2 * math.cos(angle * math.pi / 180)
    #
    #     return new_x2, new_y2
