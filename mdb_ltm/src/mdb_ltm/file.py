"""
MDB.

https://github.com/GII/MDB
"""

# Standard imports
from math import isclose

# Library imports
import yaml
import yamlloader


class File(object):
    """A MDB file."""

    def __init__(self, **kwargs):
        """Init attributes when a new object is created."""
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
        if self.file_object:
            self.file_object.close()


class FileGoodness(File):
    """A file where several goodness statistics about an experiment are stored."""

    def write_header(self):
        """Write the header of the file."""
        super().write_header()
        self.file_object.write(
            "Iteration\tGoal\tWorld\tReward\tPolicy\tSensorial changes\tC-nodes\n"
        )

    def write(self):
        """Write statistics data."""
        self.file_object.write(
            str(self.ltm.iteration)
            + "\t"
            + (self.ltm.current_goal.ident if self.ltm.current_goal else "None")
            + "\t"
            + self.ltm.current_world
            + "\t"
            + str(self.ltm.current_reward)
            + "\t"
            + self.ltm.current_policy.ident
            + "\t"
            + str(self.ltm.sensorial_changes())
            + "\t"
            + str(len(self.ltm.c_nodes))
            + "\n"
        )


class FilePNodesSuccess(File):
    """A file that records wether a P-node's activation has been successful or not."""

    def write_header(self):
        """Write the header of the file."""
        super().write_header()
        self.file_object.write("Iteration\tIdent\tSuccess\n")

    def write(self):
        """Write success."""
        for pnode in self.ltm.p_nodes:
            if pnode.successful_activation is not None:
                self.file_object.write(
                    str(self.ltm.iteration)
                    + "\t"
                    + pnode.ident
                    + "\t"
                    + str(pnode.successful_activation)
                    + "\n"
                )
                pnode.successful_activation = None


class FilePNodes(File):
    """A file where pnodes points and anti-points are stored."""

    def write_header(self):
        """Write the header of the file."""
        super().write_header()
        self.file_object.write("Iteration\tIdent\t")
        i = 0
        for perception in self.ltm.perceptions.values():
            for perception_item in perception.value:
                for key in perception_item:
                    self.file_object.write(f"{i}_{perception.ident}_{key}\t")
                i += 1
        self.file_object.write("Confidence\n")

    def write(self, force=False):
        """Write P-nodes."""
        if (self.ltm.iteration % self.data == 0) or force:
            for pnode in self.ltm.p_nodes:
                for space in pnode.spaces:
                    for point, confidence in zip(
                        space.members[0 : space.size], space.memberships[0 : space.size]
                    ):
                        self.file_object.write(
                            str(self.ltm.iteration) + "\t" + pnode.ident + "\t"
                        )
                        for sensor in point:
                            self.file_object.write(str(sensor) + "\t")
                        self.file_object.write(str(confidence) + "\n")

    def close(self):
        """Close de underlying file."""
        self.write(force=True)
        super().close()


class FileGoals(File):
    """A file that stores goals information (right now, only value functions)."""

    def write(self, force=False):
        """Write value functions."""
        if (self.ltm.iteration % self.data == 0) or force:
            for goal in self.ltm.goals:
                for value_function in goal.value_functions:
                    self.file_object.write(str(self.ltm.iteration) + " => ")
                    for subgoal in value_function:
                        self.file_object.write(subgoal.ident + "\t")
                    self.file_object.write("\n")

    def close(self):
        """Close de underlying file."""
        self.write(force=True)
        super().close()


class FileLTMDump(File):
    """A file that stores a memory dump of a LTM."""

    def write_header(self):
        """Write the header of the file."""

    def write(self):
        """Do the LTM dump."""
        self.ltm.iteration += 1
        file_name = self.file_name + "_" + str(self.ltm.iteration) + ".yaml"
        yaml.dump(
            self.ltm,
            open(file_name, "w", encoding="utf-8"),
            Dumper=yamlloader.ordereddict.CDumper,
            allow_unicode=True,
        )
        self.ltm.iteration -= 1

    def close(self):
        """Close de underlying file."""
        self.write()
        super().close()


class FileLTMDumpWhenReward(FileLTMDump):
    """A file that stores a memory dump of a LTM when a maximum reward is obtained."""

    def write(self):
        """Do the LTM dump."""
        if isclose(self.ltm.reward, 1.0):
            super().write()


class FileLTMPeriodicDump(FileLTMDump):
    """A file that stores a memory dump of a LTM each x iterations."""

    def write(self, force=False):
        """Do the LTM dump."""
        if (
            (self.ltm.iteration > 0) and (self.ltm.iteration % self.data == 0)
        ) or force:
            super().write()
