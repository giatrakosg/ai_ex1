# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util
from sets import Set

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    e = Directions.EAST 
    n = Directions.NORTH

    closed = Set()  # we use a set to keep track of the visited nodes
    paths = {} # Dict with state as key and path to state as value
    fringe = util.Stack() # we use a stack to simulate a dfs
    start = problem.getStartState()
    fringe.push(start)
    paths[start] = []
    while True:
        if fringe.isEmpty():
            return []
        node = fringe.pop() # current node
        if problem.isGoalState(node) :
            return paths[node]
        if node not in closed :
            closed.add(node)
            neighbors = problem.getSuccessors(node)
            #returns type (state,action,stepcost)
            for i in neighbors:
                # add the direction to the path already constructed for the parent
                paths[i[0]] = paths[node] + [i[1]]
                # add the state to the qeueu
                fringe.push(i[0])
    

def breadthFirstSearch(problem):

    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    e = Directions.EAST 
    n = Directions.NORTH

    closed = Set()  # we use a set to keep track of the visited nodes
    paths = {} # Dict with state as key and path to state as value
    fringe = util.Queue() # we use a stack to simulate a dfs
    start = problem.getStartState()
    fringe.push(start)
    paths[start] = []
    while True:
        if fringe.isEmpty():
            return []
        node = fringe.pop() # current node
        if problem.isGoalState(node) :
            return paths[node]
        if node not in closed :
            closed.add(node)
            neighbors = problem.getSuccessors(node)
            #returns type (state,action,stepcost)
            for i in neighbors:
                if i[0] not in fringe.list :
                    if i[0] not in closed:
                        fringe.push(i[0])
                        paths[i[0]] = paths[node] + [i[1]]
    util.raiseNotDefined()

def uniformCostSearch(problem):

    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    e = Directions.EAST 
    n = Directions.NORTH

    closed = Set()  # we use a set to keep track of the visited nodes
    paths = {} # Dict with state as key and path to state as value
    fringe = util.PriorityQueue() # we use a stack to simulate a dfs
    start = problem.getStartState()
    fringe.push(start,0)
    paths[start] = ([],0)

    while True:
        if fringe.isEmpty():
            return []
        node = fringe.pop() # current node
        if problem.isGoalState(node) :
            return paths[node][0]
        if node not in closed :
            closed.add(node)
            neighbors = problem.getSuccessors(node)
            #returns type (state,action,stepcost)
            for i in neighbors:
                if i[0] not in closed:
                    #if i[0] not in paths:
                    cost2node = paths[node][1]
                    cost2i = cost2node + i[2]
                    path2i = paths[node][0] + [i[1]]
                    if i[0] in paths:
                        if paths[i[0]][1] > cost2i :
                            paths[i[0]] = (path2i,cost2i)
                            fringe.update(i[0],cost2i)
                        else:
                            continue
                    else:
                        paths[i[0]] = (path2i,cost2i)
                        fringe.push(i[0],cost2i)

    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"


    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    e = Directions.EAST 
    n = Directions.NORTH

    closed = Set()  # we use a set to keep track of the visited nodes
    paths = {} # Dict with state as key and path to state as value
    fringe = util.PriorityQueue() # we use a qeueu to simulate a bfs
    start = problem.getStartState()
    fringe.push(start,0)
    paths[start] = []
    while True:
        if fringe.isEmpty():
            return []
        node = fringe.pop() # current node
        if problem.isGoalState(node) :
            return paths[node]
        if node not in closed :
            closed.add(node)
            neighbors = problem.getSuccessors(node)
            #returns type (state,action,stepcost)
            for i in neighbors:
                # add the direction to the path already constructed for the parent
                paths[i[0]] = paths[node] + [i[1]]
                # add the state to the qeueu with the cost 
                fringe.push(i[0],i[2] + heuristic(i[0],problem))
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
