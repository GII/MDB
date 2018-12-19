"""
The shiny, all new, MDB 3.0.

Available from (we are still thinking about this...)
Distributed under the (yes, we are still thinking about this too...).
"""

from __future__ import (absolute_import, division, print_function, unicode_literals)
from builtins import * #noqa
import rospy  # ROS

from matplotlib import pyplot as plt

from mdb_motiven.distances_certainty import DistancesCertainty


class Correlations(object):
    """
    Class that represents the Correlations module.

    This module identifies new correlations and contains the set of existing correlations.
    It contains the Correlation Evaluator that is an algorithm that has to be executed each time a trace is added
    to the Traces Memory and searches for possible correlations to be stored in the Traces Correlation Memory.
    It also has the Distances Certainty module that makes possible the creation of certainty maps using the traces
    stored as positive-traces, negative-traces and weak-traces, which aim is to obtain the certainty value for a
    point p given.
    """

    def __init__(self, rewardAssigner, goal_id):
        self.n_sensor = 10  # Number of sensors. Useful to know how many possible correlations there are
        self.min_ep = 2#5  # Minimum number of episodes to consider the correlation possible
        self.same_values_accepted = 1  # Number of sensor admitted to be equal

        # Correlations Traces Memories and certainty evaluators
        # Hacer esto dinamico en funcion del numero de sensores
        self.S1_pos = DistancesCertainty()
        self.S1_neg = DistancesCertainty()
        self.S2_neg = DistancesCertainty()
        self.S2_pos = DistancesCertainty()
        self.S3_neg = DistancesCertainty()
        self.S3_pos = DistancesCertainty()
        self.S4_pos = DistancesCertainty()
        self.S4_neg = DistancesCertainty()
        self.S5_neg = DistancesCertainty()
        self.S5_pos = DistancesCertainty()
        self.S6_neg = DistancesCertainty()
        self.S6_pos = DistancesCertainty()
        self.S7_neg = DistancesCertainty()
        self.S7_pos = DistancesCertainty()
        self.S8_neg = DistancesCertainty()
        self.S8_pos = DistancesCertainty()
        self.S9_neg = DistancesCertainty()
        self.S9_pos = DistancesCertainty()
        self.S10_neg = DistancesCertainty()
        self.S10_pos = DistancesCertainty()

        self.corr_active = 0  # 1 - Sensor 1, 2 - Sensor 2, ... n- sensor n, 0 - no hay correlacion
        self.corr_type = ''  # 'pos' - Correlacion positiva, 'neg' - Correlacion negativa, '' - no hay correlacion
        self.corr_threshold = 0.1  # Threshold to know when to consider Extrinsic Motivation and when Intrinsic

        self.established = 0
        self.corr_established = 0
        self.corr_established_type = ''

        self.i_reward = rewardAssigner  # Valor que indica el indice de la correlacion encargada de
        # asignarle el reward. Si el indice es null, sera el escenario el encargado de hacerlo

        ##########
        self.i_reward_assigned = 0  # Valor para conocer si ya le ha sido asignada una correlacion asignadora de reward
        ##########
        #########
        self.goal = goal_id  # Numero o id del goal al que esta asociada la SUR
        #########

        self.Tb = 8  # 20#8  # Number of goals without antitraces needed to consider the correlation established

        self.figure = plt.figure()
        # self.figure.canvas.set_window_title('PRUEBA')

    def correlationEvaluator(self, Trace):
        """This method evaluates the possible correlations existing in a trace T and save them in the proper Correlation
        Traces Memory Buffer

        Keyword arguments:
        Trace -- List of tuples, it is a List of episodes-sensorization(tuples)
        """
        # print "Correlation evaluator"
        # print "len trace: ", len(Trace)
        if len(Trace) >= self.min_ep:
            for i in range(self.n_sensor):
                p_corr = 1  # Positive correlation
                n_corr = 1  # Negative correlation
                same_value = 0  # Number of times a sensor has the same value in two consecutive episodes
                for j in reversed(list(range(len(Trace) - self.min_ep, len(Trace)))):
                    # No es necesario llegar al 0 porque estoy contemplandolo ya en el [j-1]
                    if p_corr:  # The case when positive correlation is active
                        if Trace[j][i] > Trace[j - 1][i]:
                            n_corr = 0  # Negative correlation is no longer possible for this sensor
                        elif Trace[j][i] < Trace[j - 1][i]:
                            p_corr = 0  # Positive correlation is no longer possible for this sensor
                        else:  # Trace[j][i]=Trace[j-1][i]
                            same_value += 1
                            if same_value > self.same_values_accepted:
                                n_corr = 0
                                p_corr = 0
                    elif n_corr:  # The case when negative correlation is active
                        if Trace[j][i] > Trace[j - 1][i]:
                            n_corr = 0  # Negative correlation is no longer possible for this sensor
                        elif Trace[j][i] < Trace[j - 1][i]:
                            p_corr = 0  # Positive correlation is no longer possible for this sensor
                        else:  # Trace[j][i]=Trace[j-1][i]
                            same_value += 1
                            if same_value > self.same_values_accepted:
                                n_corr = 0
                                p_corr = 0
                # If there is a correlation, save it in the pertinent correlation trace memory
                if p_corr:  # Si esto esta bien corregirlo, porque es redundante
                    self.addWeakTrace(Trace, i + 1, 'pos')
                elif n_corr:
                    self.addWeakTrace(Trace, i + 1, 'neg')

    def getActiveCorrelation(self, p, active_goal):
        """
        # Este metodo despues ira dentro del motivation manager, y lo de correlaciones importado alli tambien (dentro de un modulo que sea modelos de utilidad)

        # Evaluo la certeza del nuevo punto en todas las correlaciones para ver si pertenece a alguna
        # Si es mayor que un umbral para alguna de ellas, considero la mayor y si hay empate, una al azar
        # Si es menor que el umbral, consireo la motivacion intrinseca
        :param p:
        :return:
        """
        if active_goal == self.goal:
            c1_pos = self.S1_pos.getCertaintyValue(p)
            c1_neg = self.S1_neg.getCertaintyValue(p)
            c2_pos = self.S2_pos.getCertaintyValue(p)
            c2_neg = self.S2_neg.getCertaintyValue(p)
            c3_pos = self.S3_pos.getCertaintyValue(p)
            c3_neg = self.S3_neg.getCertaintyValue(p)
            c4_pos = self.S4_pos.getCertaintyValue(p)
            c4_neg = self.S4_neg.getCertaintyValue(p)
            c5_pos = self.S5_pos.getCertaintyValue(p)
            c5_neg = self.S5_neg.getCertaintyValue(p)
            c6_pos = self.S6_pos.getCertaintyValue(p)
            c6_neg = self.S6_neg.getCertaintyValue(p)
            c7_pos = self.S7_pos.getCertaintyValue(p)
            c7_neg = self.S7_neg.getCertaintyValue(p)
            c8_pos = self.S8_pos.getCertaintyValue(p)
            c8_neg = self.S8_neg.getCertaintyValue(p)
            c9_pos = self.S9_pos.getCertaintyValue(p)
            c9_neg = self.S9_neg.getCertaintyValue(p)
            c10_pos = self.S10_pos.getCertaintyValue(p)
            c10_neg = self.S10_neg.getCertaintyValue(p)

            # n_c1_pos = self.S1_pos.numberOfGoalsWithoutAntiTraces
            # n_c1_neg = self.S1_neg.numberOfGoalsWithoutAntiTraces
            # n_c2_pos = self.S2_pos.numberOfGoalsWithoutAntiTraces
            # n_c2_neg = self.S2_neg.numberOfGoalsWithoutAntiTraces
            # n_c3_pos = self.S3_pos.numberOfGoalsWithoutAntiTraces
            # n_c3_neg = self.S3_neg.numberOfGoalsWithoutAntiTraces

            if self.established:  ### Poido non estar de acordo en esto
                # j = (n_c1_pos, n_c1_neg, n_c2_pos, n_c2_neg, n_c3_pos, n_c3_neg).index(
                #     max(n_c1_pos, n_c1_neg, n_c2_pos, n_c2_neg, n_c3_pos,
                #         n_c3_neg))  # Save index of the correlated correlation
                # if self.corr_threshold > (c1_pos, c1_neg, c2_pos, c2_neg, c3_pos, c3_neg)[
                #     j]:  # Si el umbral es mayor que el valor de certeza de la correlacion consolidada
                #     self.corr_active = 0
                #     self.corr_type = ''
                # else:
                #     i = (n_c1_pos, n_c1_neg, n_c2_pos, n_c2_neg, n_c3_pos, n_c3_neg).index(
                #         max(n_c1_pos, n_c1_neg, n_c2_pos, n_c2_neg, n_c3_pos,
                #             n_c3_neg))  # Redundante, ya podria usar el indice j de arriba
                #     if i < 2:
                #         self.corr_active = 1  # Sensor 1
                #     elif i < 4:
                #         self.corr_active = 2  # Sensor 2
                #     else:
                #         self.corr_active = 3  # Sensor 3
                #     if i % 2 == 0:  # Posicion par
                #         self.corr_type = 'pos'
                #     else:
                #         self.corr_type = 'neg'
                if self.corr_established == 1:
                    if self.corr_established_type == 'pos':
                        j = 0
                    else:
                        j = 1
                elif self.corr_established == 2:
                    if self.corr_established_type == 'pos':
                        j = 2
                    else:
                        j = 3
                elif self.corr_established == 3:
                    if self.corr_established_type == 'pos':
                        j = 4
                    else:
                        j = 5
                elif self.corr_established == 4:
                    if self.corr_established_type == 'pos':
                        j = 6
                    else:
                        j = 7
                elif self.corr_established == 5:
                    if self.corr_established_type == 'pos':
                        j = 8
                    else:
                        j = 9
                elif self.corr_established == 6:
                    if self.corr_established_type == 'pos':
                        j = 10
                    else:
                        j = 11
                elif self.corr_established == 7:
                    if self.corr_established_type == 'pos':
                        j = 12
                    else:
                        j = 13
                elif self.corr_established == 8:
                    if self.corr_established_type == 'pos':
                        j = 14
                    else:
                        j = 15
                elif self.corr_established == 9:
                    if self.corr_established_type == 'pos':
                        j = 16
                    else:
                        j = 17
                else:
                    if self.corr_established_type == 'pos':
                        j = 18
                    else:
                        j = 19
                if self.corr_threshold > (
                        c1_pos, c1_neg, c2_pos, c2_neg, c3_pos, c3_neg, c4_pos, c4_neg, c5_pos, c5_neg, c6_pos, c6_neg,
                        c7_pos,
                        c7_neg, c8_pos, c8_neg, c9_pos, c9_neg, c10_pos, c10_neg)[
                    j]:  # Si el umbral es mayor que el valor de certeza de la correlacion consolidada
                    self.corr_active = 0
                    self.corr_type = ''
                else:
                    self.corr_active = self.corr_established
                    self.corr_type = self.corr_established_type
            else:
                if self.corr_threshold > max(
                        c1_pos, c1_neg, c2_pos, c2_neg, c3_pos, c3_neg, c4_pos, c4_neg, c5_pos, c5_neg, c6_pos, c6_neg,
                        c7_pos,
                        c7_neg, c8_pos, c8_neg, c9_pos, c9_neg, c10_pos, c10_neg):
                    self.corr_active = 0  # Al no haber correlacion activa doy por hecho que se usa la motivInt
                    self.corr_type = ''
                else:
                    # Guardo posicion valor maximo
                    i = (
                        c1_pos, c1_neg, c2_pos, c2_neg, c3_pos, c3_neg, c4_pos, c4_neg, c5_pos, c5_neg, c6_pos, c6_neg,
                        c7_pos,
                        c7_neg, c8_pos, c8_neg, c9_pos, c9_neg, c10_pos, c10_neg).index(
                        max(
                        c1_pos, c1_neg, c2_pos, c2_neg, c3_pos, c3_neg, c4_pos, c4_neg, c5_pos, c5_neg, c6_pos, c6_neg,
                        c7_pos,
                        c7_neg, c8_pos, c8_neg, c9_pos, c9_neg, c10_pos, c10_neg))
                    if i < 2:
                        self.corr_active = 1  # Sensor 1
                    elif i < 4:
                        self.corr_active = 2  # Sensor 2
                    elif i < 6:
                        self.corr_active = 3  # Sensor 3
                    elif i < 8:
                        self.corr_active = 4  # Sensor 4
                    elif i < 10:
                        self.corr_active = 5  # Sensor 5
                    elif i < 12:
                        self.corr_active = 6  # Sensor 6
                    elif i < 14:
                        self.corr_active = 7  # Sensor 7
                    elif i < 16:
                        self.corr_active = 8  # Sensor 8
                    elif i < 18:
                        self.corr_active = 9  # Sensor 9
                    else:
                        self.corr_active = 10  # Sensor 10
                    if i % 2 == 0:  # Posicion par
                        self.corr_type = 'pos'
                    else:
                        self.corr_type = 'neg'
                        # certainty_value = max(c1_pos, c1_neg, c2_pos, c2_neg, c3_pos, c3_neg)
                        # return self.corr_active, self.corr_type  # , certainty_value
        else:  # if the goal associated with the SUR is deactivated, this SUR has certainty 0
            self.corr_active = 0
            self.corr_type = ''

        return self.corr_active, self.corr_type  # , certainty_value

    def getCertainty(self, p, active_goal):
        """This method provides the maximum certainty value of the correlations

        :param p:
        :return:
        """
        if active_goal == self.goal:
            c1_pos = self.S1_pos.getCertaintyValue(p)
            c1_neg = self.S1_neg.getCertaintyValue(p)
            c2_pos = self.S2_pos.getCertaintyValue(p)
            c2_neg = self.S2_neg.getCertaintyValue(p)
            c3_pos = self.S3_pos.getCertaintyValue(p)
            c3_neg = self.S3_neg.getCertaintyValue(p)
            c4_pos = self.S4_pos.getCertaintyValue(p)
            c4_neg = self.S4_neg.getCertaintyValue(p)
            c5_pos = self.S5_pos.getCertaintyValue(p)
            c5_neg = self.S5_neg.getCertaintyValue(p)
            c6_pos = self.S6_pos.getCertaintyValue(p)
            c6_neg = self.S6_neg.getCertaintyValue(p)
            c7_pos = self.S7_pos.getCertaintyValue(p)
            c7_neg = self.S7_neg.getCertaintyValue(p)
            c8_pos = self.S8_pos.getCertaintyValue(p)
            c8_neg = self.S8_neg.getCertaintyValue(p)
            c9_pos = self.S9_pos.getCertaintyValue(p)
            c9_neg = self.S9_neg.getCertaintyValue(p)
            c10_pos = self.S10_pos.getCertaintyValue(p)
            c10_neg = self.S10_neg.getCertaintyValue(p)

            if self.established:
                # n_c1_pos = self.S1_pos.numberOfGoalsWithoutAntiTraces
                # n_c1_neg = self.S1_neg.numberOfGoalsWithoutAntiTraces
                # n_c2_pos = self.S2_pos.numberOfGoalsWithoutAntiTraces
                # n_c2_neg = self.S2_neg.numberOfGoalsWithoutAntiTraces
                # n_c3_pos = self.S3_pos.numberOfGoalsWithoutAntiTraces
                # n_c3_neg = self.S3_neg.numberOfGoalsWithoutAntiTraces
                #
                # j = (n_c1_pos, n_c1_neg, n_c2_pos, n_c2_neg, n_c3_pos, n_c3_neg).index(
                #     max(n_c1_pos, n_c1_neg, n_c2_pos, n_c2_neg, n_c3_pos,
                #         n_c3_neg))  # Save index of the correlated correlation
                #
                # certainty_value = (c1_pos, c1_neg, c2_pos, c2_neg, c3_pos, c3_neg)[
                #     j]  # Certainty value of the correlated correlation
                if self.corr_established == 1:
                    if self.corr_established_type == 'pos':
                        j = 0
                    else:
                        j = 1
                elif self.corr_established == 2:
                    if self.corr_established_type == 'pos':
                        j = 2
                    else:
                        j = 3
                elif self.corr_established == 3:
                    if self.corr_established_type == 'pos':
                        j = 4
                    else:
                        j = 5
                elif self.corr_established == 4:
                    if self.corr_established_type == 'pos':
                        j = 6
                    else:
                        j = 7
                elif self.corr_established == 5:
                    if self.corr_established_type == 'pos':
                        j = 8
                    else:
                        j = 9
                elif self.corr_established == 6:
                    if self.corr_established_type == 'pos':
                        j = 10
                    else:
                        j = 11
                elif self.corr_established == 7:
                    if self.corr_established_type == 'pos':
                        j = 12
                    else:
                        j = 13
                elif self.corr_established == 8:
                    if self.corr_established_type == 'pos':
                        j = 14
                    else:
                        j = 15
                elif self.corr_established == 9:
                    if self.corr_established_type == 'pos':
                        j = 16
                    else:
                        j = 17
                else:
                    if self.corr_established_type == 'pos':
                        j = 18
                    else:
                        j = 19
                certainty_value = (
                    c1_pos, c1_neg, c2_pos, c2_neg, c3_pos, c3_neg, c4_pos, c4_neg, c5_pos, c5_neg, c6_pos, c6_neg,
                    c7_pos,
                    c7_neg, c8_pos, c8_neg, c9_pos, c9_neg, c10_pos, c10_neg)[j]
            else:
                certainty_value = max(c1_pos, c1_neg, c2_pos, c2_neg, c3_pos, c3_neg, c4_pos, c4_neg, c5_pos, c5_neg,
                                      c6_pos, c6_neg, c7_pos,
                                      c7_neg, c8_pos, c8_neg, c9_pos, c9_neg, c10_pos, c10_neg)
            return certainty_value
        else:  # if the goal associated with the SUR is deactivated, this SUR has certainty 0
            return 0

    def addTrace(self, Trace, sensor, corr_type):
        if not self.established:
            rospy.logdebug('New ' + str(corr_type) + ' Trace in sensor ' + str(sensor))
            # Guardo solo hasta donde se cumple la correlacion
            for i in reversed(list(range(len(Trace)))):
                if corr_type == 'neg':
                    if Trace[i][sensor - 1] >= Trace[i - 1][sensor - 1]:
                        break
                elif corr_type == 'pos':
                    if Trace[i][sensor - 1] <= Trace[i - 1][sensor - 1]:
                        break
            if sensor == 1:
                if corr_type == 'pos':
                    self.S1_pos.addTraces(Trace[i:])
                elif corr_type == 'neg':
                    self.S1_neg.addTraces(Trace[i:])
            elif sensor == 2:
                if corr_type == 'pos':
                    self.S2_pos.addTraces(Trace[i:])
                elif corr_type == 'neg':
                    self.S2_neg.addTraces(Trace[i:])
            elif sensor == 3:
                if corr_type == 'pos':
                    self.S3_pos.addTraces(Trace[i:])
                elif corr_type == 'neg':
                    self.S3_neg.addTraces(Trace[i:])
            elif sensor == 4:
                if corr_type == 'pos':
                    self.S4_pos.addTraces(Trace[i:])
                elif corr_type == 'neg':
                    self.S4_neg.addTraces(Trace[i:])
            elif sensor == 5:
                if corr_type == 'pos':
                    self.S5_pos.addTraces(Trace[i:])
                elif corr_type == 'neg':
                    self.S5_neg.addTraces(Trace[i:])
            elif sensor == 6:
                if corr_type == 'pos':
                    self.S6_pos.addTraces(Trace[i:])
                elif corr_type == 'neg':
                    self.S6_neg.addTraces(Trace[i:])
            elif sensor == 7:
                if corr_type == 'pos':
                    self.S7_pos.addTraces(Trace[i:])
                elif corr_type == 'neg':
                    self.S7_neg.addTraces(Trace[i:])
            elif sensor == 8:
                if corr_type == 'pos':
                    self.S8_pos.addTraces(Trace[i:])
                elif corr_type == 'neg':
                    self.S8_neg.addTraces(Trace[i:])
            elif sensor == 9:
                if corr_type == 'pos':
                    self.S9_pos.addTraces(Trace[i:])
                elif corr_type == 'neg':
                    self.S9_neg.addTraces(Trace[i:])
            elif sensor == 10:
                if corr_type == 'pos':
                    self.S10_pos.addTraces(Trace[i:])
                elif corr_type == 'neg':
                    self.S10_neg.addTraces(Trace[i:])

            # Check if the correlation is established (it could only happen after adding a trace)
            self.isCorrelationEstablished()

    def addAntiTrace(self, Trace, sensor, corr_type):
        # Filtro aqui para guardar los valores obtenidos con motivacion extrinseca
        # if not self.established:
        rospy.logdebug('New ' + str(corr_type) + ' AntiTrace in sensor ' + str(sensor))
        if sensor == 1:
            if corr_type == 'pos':
                self.S1_pos.addAntiTraces(Trace)
            elif corr_type == 'neg':
                self.S1_neg.addAntiTraces(Trace)
        elif sensor == 2:
            if corr_type == 'pos':
                self.S2_pos.addAntiTraces(Trace)
            elif corr_type == 'neg':
                self.S2_neg.addAntiTraces(Trace)
        elif sensor == 3:
            if corr_type == 'pos':
                self.S3_pos.addAntiTraces(Trace)
            elif corr_type == 'neg':
                self.S3_neg.addAntiTraces(Trace)
        elif sensor == 4:
            if corr_type == 'pos':
                self.S4_pos.addAntiTraces(Trace)
            elif corr_type == 'neg':
                self.S4_neg.addAntiTraces(Trace)
        elif sensor == 5:
            if corr_type == 'pos':
                self.S5_pos.addAntiTraces(Trace)
            elif corr_type == 'neg':
                self.S5_neg.addAntiTraces(Trace)
        elif sensor == 6:
            if corr_type == 'pos':
                self.S6_pos.addAntiTraces(Trace)
            elif corr_type == 'neg':
                self.S6_neg.addAntiTraces(Trace)
        elif sensor == 7:
            if corr_type == 'pos':
                self.S7_pos.addAntiTraces(Trace)
            elif corr_type == 'neg':
                self.S7_neg.addAntiTraces(Trace)
        elif sensor == 8:
            if corr_type == 'pos':
                self.S8_pos.addAntiTraces(Trace)
            elif corr_type == 'neg':
                self.S8_neg.addAntiTraces(Trace)
        elif sensor == 9:
            if corr_type == 'pos':
                self.S9_pos.addAntiTraces(Trace)
            elif corr_type == 'neg':
                self.S9_neg.addAntiTraces(Trace)
        elif sensor == 10:
            if corr_type == 'pos':
                self.S10_pos.addAntiTraces(Trace)
            elif corr_type == 'neg':
                self.S10_neg.addAntiTraces(Trace)

    def addWeakTrace(self, Trace, sensor, corr_type):
        # plt.figure()
        if not self.established:
            rospy.logdebug('New ' + str(corr_type) + ' WeakTrace in sensor ' + str(sensor))
            # Guardo solo hasta donde se cumple la correlacion
            for i in reversed(list(range(len(Trace)))):
                if corr_type == 'neg':
                    if Trace[i][sensor - 1] >= Trace[i - 1][sensor - 1]:
                        break
                elif corr_type == 'pos':
                    if Trace[i][sensor - 1] <= Trace[i - 1][sensor - 1]:
                        break
            if sensor == 1:
                if corr_type == 'pos':
                    self.S1_pos.addWeakTraces(Trace[i:])
                elif corr_type == 'neg':
                    self.S1_neg.addWeakTraces(Trace[i:])
            elif sensor == 2:
                if corr_type == 'pos':
                    self.S2_pos.addWeakTraces(Trace[i:])
                elif corr_type == 'neg':
                    self.S2_neg.addWeakTraces(Trace[i:])
            elif sensor == 3:
                if corr_type == 'pos':
                    self.S3_pos.addWeakTraces(Trace[i:])
                elif corr_type == 'neg':
                    self.S3_neg.addWeakTraces(Trace[i:])
            elif sensor == 4:
                if corr_type == 'pos':
                    self.S4_pos.addWeakTraces(Trace[i:])
                elif corr_type == 'neg':
                    self.S4_neg.addWeakTraces(Trace[i:])
            elif sensor == 5:
                if corr_type == 'pos':
                    self.S5_pos.addWeakTraces(Trace[i:])
                elif corr_type == 'neg':
                    self.S5_neg.addWeakTraces(Trace[i:])
            elif sensor == 6:
                if corr_type == 'pos':
                    self.S6_pos.addWeakTraces(Trace[i:])
                elif corr_type == 'neg':
                    self.S6_neg.addWeakTraces(Trace[i:])
            elif sensor == 7:
                if corr_type == 'pos':
                    self.S7_pos.addWeakTraces(Trace[i:])
                elif corr_type == 'neg':
                    self.S7_neg.addWeakTraces(Trace[i:])
            elif sensor == 8:
                if corr_type == 'pos':
                    self.S8_pos.addWeakTraces(Trace[i:])
                elif corr_type == 'neg':
                    self.S8_neg.addWeakTraces(Trace[i:])
            elif sensor == 9:
                if corr_type == 'pos':
                    self.S9_pos.addWeakTraces(Trace[i:])
                elif corr_type == 'neg':
                    self.S9_neg.addWeakTraces(Trace[i:])
            elif sensor == 10:
                if corr_type == 'pos':
                    self.S10_pos.addWeakTraces(Trace[i:])
                elif corr_type == 'neg':
                    self.S10_neg.addWeakTraces(Trace[i:])

    def isCorrelationEstablished(self):
        corr1_pos = self.S1_pos.numberOfGoalsWithoutAntiTraces
        corr1_neg = self.S1_neg.numberOfGoalsWithoutAntiTraces
        corr2_pos = self.S2_pos.numberOfGoalsWithoutAntiTraces
        corr2_neg = self.S2_neg.numberOfGoalsWithoutAntiTraces
        corr3_pos = self.S3_pos.numberOfGoalsWithoutAntiTraces
        corr3_neg = self.S3_neg.numberOfGoalsWithoutAntiTraces
        corr4_pos = self.S4_pos.numberOfGoalsWithoutAntiTraces
        corr4_neg = self.S4_neg.numberOfGoalsWithoutAntiTraces
        corr5_pos = self.S5_pos.numberOfGoalsWithoutAntiTraces
        corr5_neg = self.S5_neg.numberOfGoalsWithoutAntiTraces
        corr6_pos = self.S6_pos.numberOfGoalsWithoutAntiTraces
        corr6_neg = self.S6_neg.numberOfGoalsWithoutAntiTraces
        corr7_pos = self.S7_pos.numberOfGoalsWithoutAntiTraces
        corr7_neg = self.S7_neg.numberOfGoalsWithoutAntiTraces
        corr8_pos = self.S8_pos.numberOfGoalsWithoutAntiTraces
        corr8_neg = self.S8_neg.numberOfGoalsWithoutAntiTraces
        corr9_pos = self.S9_pos.numberOfGoalsWithoutAntiTraces
        corr9_neg = self.S9_neg.numberOfGoalsWithoutAntiTraces
        corr10_pos = self.S10_pos.numberOfGoalsWithoutAntiTraces
        corr10_neg = self.S10_neg.numberOfGoalsWithoutAntiTraces

        max_traces = max(corr1_pos, corr1_neg, corr2_pos, corr2_neg, corr3_pos, corr3_neg, corr4_pos, corr4_neg,
                         corr5_pos, corr5_neg, corr6_pos, corr6_neg, corr7_pos, corr7_neg, corr8_pos, corr8_neg,
                         corr9_pos, corr9_neg, corr10_pos, corr10_neg)
        if not self.established:
            if max_traces >= self.Tb:
                self.established = 1
                i = (corr1_pos, corr1_neg, corr2_pos, corr2_neg, corr3_pos, corr3_neg, corr4_pos, corr4_neg,
                     corr5_pos, corr5_neg, corr6_pos, corr6_neg, corr7_pos, corr7_neg, corr8_pos, corr8_neg,
                     corr9_pos, corr9_neg, corr10_pos, corr10_neg).index(max_traces)
                if i < 2:
                    self.corr_established = 1  # Sensor 1
                elif i < 4:
                    self.corr_established = 2  # Sensor 2
                elif i < 6:
                    self.corr_established = 3  # Sensor 3
                elif i < 8:
                    self.corr_established = 4  # Sensor 4
                elif i < 10:
                    self.corr_established = 5  # Sensor 5
                elif i < 12:
                    self.corr_established = 6  # Sensor 6
                elif i < 14:
                    self.corr_established = 7  # Sensor 7
                elif i < 16:
                    self.corr_established = 8  # Sensor 8
                elif i < 18:
                    self.corr_established = 9  # Sensor 9
                else:
                    self.corr_established = 10  # Sensor 10
                if i % 2 == 0:  # Posicion par
                    self.corr_established_type = 'pos'
                else:
                    self.corr_established_type = 'neg'
            else:
                self.established = 0
