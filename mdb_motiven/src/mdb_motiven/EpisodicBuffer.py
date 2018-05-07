class EpisodicBuffer(object):
    """ Class that represents an Episodic Buffer
    
    This Episodic buffer (EB) stores the last episodes experienced by the robot.
    The EB has a limited capacity according to the temporal nature of the STM.
    All episodes of the same trace are kept together, so each time a new trace is
    acquired, some of the old ones could be deleted following a FIFO policy.
    
    This class implements different methods to get/set the buffer size, get its
    contents, add/remove episodes and check if the buffer is full. It also assigns
    the reward when necessary.
    """

    def __init__(self):
        self.buffer = []
        self.maxSize = 10

    def setMaxSize(self, maxSize):
        self.maxSize = maxSize

    def getMaxSize(self):
        return self.maxSize

    def getSize(self):
        return len(self.buffer)

    def getContents(self):
        return self.buffer

    def isFull(self):
        return self.getSize() >= self.maxSize

    def removeAll(self):
        del self.buffer[:]

    def addEpisode(self, episode):
        if (self.isFull()):
            self.buffer.pop(0)
        self.buffer.append(episode)

    def removeEpisode(self, index):
        self.buffer.pop(index)
