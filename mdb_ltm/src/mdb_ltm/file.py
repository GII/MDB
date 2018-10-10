"""
The shiny, all new, MDB 3.0.

Available from (we are still thinking about this...)
Copyright 2017 Richard J. Duro, Jose A. Becerra.
Distributed under the (yes, we are still thinking about this too...).
"""

import yaml


class File(object):
    """
    An MDB file.

    Attributes:

    """

    def __init__(self, **kwargs):
        """Constructor."""
        self.ident = kwargs['ident']
        self.file_name = kwargs['file_name']
        self.file_object = None
        self.data = kwargs.get('data')
        self.ltm = kwargs['ltm']
        super(File, self).__init__()

    def __getstate__(self):
        """Return the object to be serialize with PyYAML as the result of removing the unpicklable entries."""
        state = self.__dict__.copy()
        del state['file_object']
        return state

    def write_header(self):
        """Write the header of the file."""
        self.file_object = open(self.file_name, 'a')

    def close(self):
        """Close de underlying file."""
        self.file_object.close()


class FileGoodness(File):
    """A file where several goodness statistics about an experiment are stored."""

    def write_header(self):
        """Write the header of the file."""
        super(FileGoodness, self).write_header()
        self.file_object.write('Iteration\tGoal\tWorld\tReward\tPolicy\tSensorial changes\tC-nodes\n')

    def write(self):
        """Write statistics data."""
        self.file_object.write(
            str(self.ltm.iteration) + '\t' + self.ltm.current_goal.ident + '\t' + self.ltm.current_world +
            '\t' + str(self.ltm.current_success) + '\t' + self.ltm.current_policy.ident + '\t' +
            str(self.ltm.sensorial_changes()) + '\t' + str(len(self.ltm.c_nodes)) + '\n')


class FilePNodes(File):
    """A file where pnodes points and anti-points are stored."""

    def write_header(self):
        """Write the header of the file."""
        super(FilePNodes, self).write_header()
        self.file_object.write('Iteration\tIdent\t')
        for perception in self.ltm.perceptions.itervalues():
            self.file_object.write(perception.ident + '\t')
        self.file_object.write('Confidence\n')

    def write(self):
        """Write P-nodes."""
        if self.ltm.iteration % self.data == 0:
            for pnode in self.ltm.p_nodes:
                for point in xrange(0, pnode.size):
                    self.file_object.write(str(self.ltm.iteration) + '\t' + pnode.ident + '\t')
                    for perception in xrange(0, pnode.n_perceptions):
                        self.file_object.write(str(pnode.members[perception, point]) + '\t')
                    self.file_object.write(str(pnode.memberships[point]) + '\n')

    def close(self):
        """Close de underlying file."""
        for pnode in self.ltm.p_nodes:
            for point in xrange(0, pnode.size):
                self.file_object.write(str(self.ltm.iteration) + '\t' + pnode.ident + '\t')
                for perception in xrange(0, pnode.n_perceptions):
                    self.file_object.write(str(pnode.members[perception, point]) + '\t')
                self.file_object.write(str(pnode.memberships[point]) + '\n')
        super(FilePNodes, self).write_header()


class FileLTMDump(File):
    """A file that stores a memory dump of a LTM."""

    def write_header(self):
        """Write the header of the file."""
        pass

    def write(self):
        """Do the LTM dump."""
        self.ltm.iteration += 1
        file_name = self.file_name + '_' + str(self.ltm.iteration) + '.yaml'
        yaml.dump(self.ltm, open(file_name, 'w'), Dumper=yaml.CDumper)
        self.ltm.iteration -= 1

    def close(self):
        """Close de underlying file."""
        self.write()
        super(FileLTMDump, self).write_header()


class FileLTMDumpWhenReward(FileLTMDump):
    """A file that stores a memory dump of a LTM when a maximum reward is obtained."""

    def write(self):
        """Do the LTM dump."""
        if self.ltm.reward == 1.0:
            super(FileLTMDumpWhenReward, self).write()


class FileLTMPeriodicDump(FileLTMDump):
    """A file that stores a memory dump of a LTM each x iterations."""

    def write(self):
        """Do the LTM dump."""
        if (self.ltm.iteration > 0) and (self.ltm.iteration % self.data == 0):
            super(FileLTMPeriodicDump, self).write()
