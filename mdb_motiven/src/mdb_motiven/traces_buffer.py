"""
MDB.

https://github.com/GII/MDB
"""

# Python 2 compatibility imports
from __future__ import absolute_import, division, print_function, unicode_literals
from future import standard_library

standard_library.install_aliases()
from builtins import *  # noqa pylint: disable=unused-wildcard-import,wildcard-import

# MDB imports
from mdb_motiven.episodic_buffer import EpisodicBuffer


class TracesBuffer(EpisodicBuffer):

    # def __init__(self):
    #     super(TracesBuffer, self).__init__()

    def rewardAssignment(self):
        reward = 1.0
        for i in reversed(list(range(len(self.buffer)))):
            self.buffer[i][-1] = reward
            reward -= 1.0 / self.maxSize  # reward -= 1.0 / len(self.buffer)

    def getTrace(self):
        """Return the trace values needed to use in the Correlations, the sensorization in t+1"""
        Trace = []
        for i in range(len(self.buffer)):
            Trace.append(self.buffer[i][2])

        return tuple(Trace)

    def getTraceReward(self):
        """Return the trace values and they associated reward to use to train the VF network"""
        self.rewardAssignment()
        Trace = []
        for i in range(len(self.buffer)):
            Trace.append(self.buffer[i][2:])

        return tuple(Trace)

    def getAntiTrace(self):
        """Return the antitrace values needed to use in the Correlations,
        the sensorization in t+1 obtained using the extrinsic motivation"""
        for i in reversed(list(range(len(self.buffer)))):
            if self.buffer[i][4] == "Int":
                break
        antiTrace = self.buffer[i:][2]

        return tuple(antiTrace)
