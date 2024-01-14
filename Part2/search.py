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

# If you have any question, ask in Piazza or email me (mxchen21@cse.cuhk.edu.hk)
# DO NOT copy the answer from any website. You can refer to tutorial slides, but try it by yourself first! 


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""
import copy
from sys import path

# import searchAgents
import util

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
    Question 1: Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))

    You only need to submit this file. Do not change other files!
    If you finish this function, you almost finish all the questions!
    Read util.py to find suitable data structure!
    All you need is pass all the code in commands.txt
    """

    # SOLUTION 1 iterative function
    "*** YOUR CODE HERE ***"
    # S = util.Stack()
    # output = util.Stack()
    # path = []
    # visited = []
    # done = []
    # parent = {}
    # start_pt = [problem.getStartState(), '', 0]
    # S.push(start_pt)
    #
    # while not S.isEmpty():
    #     current = S.pop()
    #     if problem.isGoalState(current[0]):
    #         output.push(current)
    #         break
    #     noSuccessor = True
    #     for successor in problem.getSuccessors(current[0]):
    #         if successor[0] not in visited and successor[0] not in done:
    #             noSuccessor = False
    #             S.push(successor)
    #             parent[successor[0]] = current
    #             visited.append(successor[0])
    #     if noSuccessor:
    #         done.append(current[0])
    #
    # goal = output.pop()
    #
    # current = goal
    # while current[0] is not start_pt[0]:
    #     path.append(parent[current[0]][1])
    #     current = parent[current[0]]
    #
    # path.reverse()
    # path.remove('')
    # path.append(goal[1])
    # return path

    # SOLUTION 1 recursive function
    "*** YOUR CODE HERE ***"
    output = util.Stack()
    visited = []
    parent = {}
    path = []

    def recur_dfs(node):
        if output.isEmpty():
            visited.append(node[0])
            if problem.isGoalState(node[0]):
                output.push(node)
                return
            for successor in problem.getSuccessors(node[0]):
                if successor[0] not in visited:
                    parent[successor[0]] = node
                    recur_dfs(successor)

    start_pt = [problem.getStartState(), '', 0]

    recur_dfs(start_pt)
    goal = output.pop()

    current = goal
    while current[0] is not start_pt[0]:
        path.append(parent[current[0]][1])
        current = parent[current[0]]

    path.reverse()
    path.remove('')
    path.append(goal[1])
    return path


def breadthFirstSearch(problem):
    """Question 2: Search the shallowest nodes in the search tree first."""

    "*** YOUR CODE HERE ***"
    visited = []
    expanded = []
    queue = util.Queue()
    parent = {}
    path = []
    goal_state = util.Stack()
    start_pt = [problem.getStartState(), '', 0]
    queue.push(start_pt)

    while not queue.isEmpty():
        node = queue.pop()
        if node[0] not in visited:
            visited.append(node[0])
            if problem.isGoalState(node[0]):
                goal_state.push(node)
                break
            for successor in problem.getSuccessors(node[0]):
                if successor[0] not in expanded:
                    expanded.append(successor[0])
                    parent[successor[0]] = node
                    queue.push(successor)

    goal = goal_state.pop()

    current = goal
    while current[0] is not start_pt[0]:
        path.append(parent[current[0]][1])
        current = parent[current[0]]

    path.reverse()
    path.remove('')
    path.append(goal[1])
    return path


def uniformCostSearch(problem):
    """Question 3: Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    visited = []  # storing visited nodes
    expanded = []  # storing expanded successors
    queue = util.PriorityQueue()  # storing nodes that are to be visited
    parent = {}  # storing the parent of a node
    cost = {}  # storing the total cost of visiting a node
    path = []  # results to be returned
    goal_state = util.Stack() # storing the goal state

    start_pt = [problem.getStartState(), '']
    cost[start_pt[0]] = 0
    queue.push(start_pt, cost[start_pt[0]])

    while not queue.isEmpty():
        node = queue.pop()
        if node[0] not in visited:
            visited.append(node[0])

            if problem.isGoalState(node[0]):
                goal_state.push(node)
                break

            for successor in problem.getSuccessors(node[0]):
                new_cost = successor[2] + cost[node[0]]

                # expand the successors that have not been expanded before
                if successor[0] not in expanded:
                    expanded.append(successor[0])
                    parent[successor[0]] = node
                    cost[successor[0]] = new_cost
                    queue.update(successor[0:2], cost[successor[0]])

                # update the path if a lower-cost path is found
                if new_cost < cost[successor[0]]:
                    parent[successor[0]] = node
                    cost[successor[0]] = new_cost
                    queue.update(successor[0:2], cost[successor[0]])

    goal = goal_state.pop()

    # reconstruct the path
    current = goal
    while current[0] is not start_pt[0]:
        path.append(parent[current[0]][1])
        current = parent[current[0]]
    path.reverse()
    path.remove('')
    path.append(goal[1])

    return path

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Question 4: Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    visited = []  # storing visited nodes
    expanded = []  # storing expanded successors
    queue = util.PriorityQueue()  # storing nodes that are to be visited
    parent = {}  # storing the parent of a node
    cost = {}  # storing the total cost of visiting a node
    f = {}  # storing the cost and heuristic of a node
    path = []  # results to be returned
    goal_state = util.Stack()  # storing the goal state

    start_pt = [problem.getStartState(), '', 0]
    cost[start_pt[0]] = start_pt[2]
    f[start_pt[0]] = cost[start_pt[0]] + heuristic(start_pt[0], problem)
    queue.push(start_pt, f[start_pt[0]])

    while not queue.isEmpty():
        node = queue.pop()
        if node[0] not in visited:
            visited.append(node[0])

            if problem.isGoalState(node[0]):
                goal_state.push(node)
                break

            for successor in problem.getSuccessors(node[0]):
                new_cost = successor[2] + cost[node[0]]
                new_f = new_cost + heuristic(successor[0], problem)

                # expand the successors that have not been expanded before
                if successor[0] not in expanded:
                    expanded.append(successor[0])
                    parent[successor[0]] = node
                    cost[successor[0]] = new_cost
                    f[successor[0]] = new_f
                    queue.update(successor[0:2], f[successor[0]])

                # update the path if a lower-f path is found
                if new_f < f[successor[0]]:
                    parent[successor[0]] = node
                    cost[successor[0]] = new_cost
                    f[successor[0]] = new_f
                    queue.update(successor[0:2], f[successor[0]])

    goal = goal_state.pop()

    # reconstruct the path
    current = goal
    while current[0] is not start_pt[0]:
        path.append(parent[current[0]][1])
        current = parent[current[0]]
    path.reverse()
    path.remove('')
    path.append(goal[1])

    return path


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
