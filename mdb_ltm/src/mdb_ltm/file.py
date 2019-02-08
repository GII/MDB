"""
The shiny, all new, MDB 3.0.

Available from (we are still thinking about this...)
Copyright 2017 Richard J. Duro, Jose A. Becerra.
Distributed under the (yes, we are still thinking about this too...).
"""

from __future__ import absolute_import, division, print_function, unicode_literals
from builtins import *  # noqa
from io import open
import yaml


class File(object):
    """A MDB file."""

    def __init__(self, **kwargs):
        """Constructor."""
        self.ident = kwargs["ident"]
        self.file_name = kwargs["file_name"]
        self.file_object = None
        self.data = kwargs.get("data")
        self.ltm = kwargs["ltm"]
        super().__init__()

    def __getstate__(self):
        """Return the object to be serialize with PyYAML as the result of removing the unpicklable entries."""
        state = self.__dict__.copy()
        del state["file_object"]
        return state

    def write_header(self):
        """Write the header of the file."""
        self.file_object = open(self.file_name, "a", encoding="utf-8")

    def close(self):
        """Close de underlying file."""
        self.file_object.close()


class FileGoodness(File):
    """A file where several goodness statistics about an experiment are stored."""

    def write_header(self):
        """Write the header of the file."""
        super().write_header()
        self.file_object.write("Iteration\tGoal\tWorld\tReward\tPolicy\tSensorial changes\tC-nodes\n")

    def write(self):
        """Write statistics data."""
        if self.ltm.current_goal is not None:
            goal_name = self.ltm.current_goal.ident
        else:
            goal_name = "None"
        self.file_object.write(
            str(self.ltm.iteration)
            + "\t"
            + goal_name
            + "\t"
            + self.ltm.current_world
            + "\t"
            + str(self.ltm.current_success)
            + "\t"
            + self.ltm.current_policy.ident
            + "\t"
            + str(self.ltm.sensorial_changes())
            + "\t"
            + str(len(self.ltm.c_nodes))
            + "\n"
        )


class FilePNodes(File):
    """A file where pnodes points and anti-points are stored."""

    def write_header(self):
        """Write the header of the file."""
        super().write_header()
        self.file_object.write("Iteration\tIdent\t")
        for perception in self.ltm.perceptions.values():
            self.file_object.write(perception.ident + "\t")
        self.file_object.write("Confidence\n")

    def write(self):
        """Write P-nodes."""
        if self.ltm.iteration % self.data == 0:
            for pnode in self.ltm.p_nodes:
                for point in range(0, pnode.size):
                    self.file_object.write(str(self.ltm.iteration) + "\t" + pnode.ident + "\t")
                    for perception in range(0, pnode.n_perceptions):
                        self.file_object.write(str(pnode.members[perception, point]) + "\t")
                    self.file_object.write(str(pnode.memberships[point]) + "\n")

    def close(self):
        """Close de underlying file."""
        for pnode in self.ltm.p_nodes:
            for point in range(0, pnode.size):
                self.file_object.write(str(self.ltm.iteration) + "\t" + pnode.ident + "\t")
                for perception in range(0, pnode.n_perceptions):
                    self.file_object.write(str(pnode.members[perception, point]) + "\t")
                self.file_object.write(str(pnode.memberships[point]) + "\n")
        super().write_header()


class FileLTMDump(File):
    """A file that stores a memory dump of a LTM."""

    def write_header(self):
        """Write the header of the file."""

    def write(self):
        """Do the LTM dump."""
        self.ltm.iteration += 1
        file_name = self.file_name + "_" + str(self.ltm.iteration) + ".yaml"
        yaml.dump(self.ltm, open(file_name, "w", encoding="utf-8"), Dumper=yaml.CDumper, allow_unicode=True)
        self.ltm.iteration -= 1

    def close(self):
        """Close de underlying file."""
        self.write()
        super().write_header()


class FileLTMDumpWhenReward(FileLTMDump):
    """A file that stores a memory dump of a LTM when a maximum reward is obtained."""

    def write(self):
        """Do the LTM dump."""
        if self.ltm.reward == 1.0:
            super().write()


class FileLTMPeriodicDump(FileLTMDump):
    """A file that stores a memory dump of a LTM each x iterations."""

    def write(self):
        """Do the LTM dump."""
        if (self.ltm.iteration > 0) and (self.ltm.iteration % self.data == 0):
            super().write()
