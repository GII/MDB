class TracesMemory(object):
    """ Class that represents a Memory of Traces
    
    A trace is a list of episodes with an assigned value (expected reward)
    which are stored together.
    It distinguish between Positive Traces (named Traces), Negative Traces
    (named AntiTraces) and WeakPositive Traces (named WeakTraces)
        
    This class implements different methods to get/set the different traces
    lists, get their contents and add/remove traces.
    """

    def __init__(self):
        self.tracesList = []
        self.antiTracesList = []
        self.weakTracesList = []
        # self.listsRemoved = []
        # self.weaksRemoved = []
        # self.traceListTime = []

    def addTraces(self, traces):
        self.tracesList.append(traces)

    def addAntiTraces(self, traces):
        self.antiTracesList.append(traces)

    def addWeakTraces(self, traces):
        self.weakTracesList.append(traces)

    def getTracesList(self):
        return self.tracesList

    def getAntiTracesList(self):
        return self.antiTracesList

    def getWeakTracesList(self):
        return self.weakTracesList

    # def getListsRemoved(self):
    #     return self.listsRemoved

    # def setListsRemoved(self, listsRemoved):
    #     self.listsRemoved = listsRemoved

    # def addListsRemoved(self, listsToBeAdded):
    #     self.listsRemoved.extend(listsToBeAdded)
    #
    # def getWeaksRemoved(self):
    #     return self.weaksRemoved

        # def setWeakTracesList(self, weakTracesList):
        #     self.weakTracesList = weakTracesList

    # def addWeaksRemoved(self, weakTracesList):
    #     self.weaksRemoved.extend(weakTracesList)
