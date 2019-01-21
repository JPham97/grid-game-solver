# Jeremy Pham
# A12962840
# CSE 150 PA1

from __future__ import print_function
#Use priority queues from Python libraries, don't waste time implementing your own
from heapq import *

ACTIONS = [(0,-1),(-1,0),(0,1),(1,0)]

class Agent:
    def __init__(self, grid, start, goal, type):
        self.grid = grid
        self.previous = {}
        self.explored = []
        self.start = start 
        self.grid.nodes[start].start = True
        self.goal = goal
        self.grid.nodes[goal].goal = True
        self.new_plan(type)

    '''
    Initializes the data structures based on the search algorithm being used
    '''
    def new_plan(self, type):
        self.finished = False
        self.failed = False
        self.type = type

        # for DFS/BFS: add the starting node to the frontier and initialize the list of explored nodes
        if self.type == "dfs" :
            self.frontier = [self.start]
            self.explored = []
        elif self.type == "bfs":
            self.frontier = [self.start]
            self.explored = []

        # These algorithms utilize a priority queue and weights/costs of nodes
        elif self.type == "ucs":
            self.frontier = [(0, self.start)]
            self.explored = []

            # treat the frontier as a PQ, so it must be heapified first
            heapify(self.frontier)

        elif self.type == "astar":
            self.frontier = [(self.dist(self.start, self.goal), self.start, 0)]
            self.explored = []

            # treat the frontier as a PQ, so it must be heapified first
            heapify(self.frontier)

    def show_result(self):
        current = self.goal
        while not current == self.start:
            current = self.previous[current]
            self.grid.nodes[current].in_path = True #This turns the color of the node to red
    def make_step(self):
        if self.type == "dfs":
            self.dfs_step()
        elif self.type == "bfs":
            self.bfs_step()
        elif self.type == "ucs":
            self.ucs_step()
        elif self.type == "astar":
            self.astar_step()

    '''
    Performs a step of the DFS algorithm, which uses a stack to explore nodes in a graph
    '''
    def dfs_step(self):
        # Check to see if the frontier is initialized (should contain start node)
        # if not the first step, checks to see if there are nodes in the frontier to be explored
        if not self.frontier:
            self.failed = True
            print("no path")
            return
        current = self.frontier.pop()
        print("current node: ", current)
        # The popped node is set as "explored" and removed from the frontier
        self.grid.nodes[current].checked = True
        self.grid.nodes[current].frontier = False
        self.explored.append(current)
        children = [(current[0]+a[0], current[1]+a[1]) for a in ACTIONS]
        # iterate through all the children (up, down, left, right)
        for node in children:
            #See what happens if you disable this check here
            if node in self.explored or node in self.frontier:
                print("explored before: ", node)
                continue
            # make sure the node is within the grid by checking ranges and coordinates
            if node[0] in range(self.grid.row_range) and node[1] in range(self.grid.col_range):
                # check to see if the child is a puddle
                if self.grid.nodes[node].puddle:
                    print("puddle at: ", node)
                else:
                    # this child is NOT a puddle, so record the previous node that we came from to form a path
                    self.previous[node] = current
                    if node == self.goal:
                        self.finished = True
                        return
                    else:
                        # add this child to the frontier data structure if it is not the goal
                        self.frontier.append(node)
                        # set a boolean to True to denote that this child is in the frontier
                        self.grid.nodes[node].frontier = True
            else:
                print("out of range: ", node)
    '''
    Performs a step of the BFS algorithm, which uses a queue to explore nodes in a graph
    '''
    def bfs_step(self):
        # Check to see if the frontier is not empty
        if not self.frontier:
            self.failed = True
            print("no path")
            return
        current = self.frontier.pop(0)
        print("current node: ", current)
        # set this node as "explored"
        self.grid.nodes[current].checked = True
        self.grid.nodes[current].frontier = False
        self.explored.append(current)
        children = [(current[0]+a[0], current[1]+a[1]) for a in ACTIONS]
        # loop through the children
        for node in children:
            # do not explore if already checked
            if node in self.explored or node in self.frontier:
                print("explored before: ", node)
                continue
            # make sure node is within the grid to prevent going outside the grid
            if node[0] in range(self.grid.row_range) and node[1] in range(self.grid.col_range):
                # cannot traverse puddles
                if self.grid.nodes[node].puddle:
                    print("puddle at: ", node)
                else: 
                    # set the previous node to form a path (the node traversed to reach this child)
                    self.previous[node] = current
                    if node == self.goal:
                        self.finished = True
                        return
                    else:
                        # add this current child to the frontier
                        self.frontier.append(node)
                        # set the boolean to denote this child as in the frontier
                        self.grid.nodes[node].frontier = True

    '''
    Performs a step of the UCS algorithm (Dijkstra's), which uses a priority queue to find the next
    node with the smallest cost
    '''
    def ucs_step(self):
        #[Hint] you can get the cost of a node by node.cost()

        # Check to see if the frontier is not empty
        if not self.frontier:
            self.failed = True
            print("no path")
            return

        # Pop the node with the smallest cost from the heap (should be a pair (Cost, Node))
        current_pair = heappop(self.frontier)
        current = current_pair[1]
        print("current node: ", current)
        # set this node as "explored"
        self.grid.nodes[current].checked = True
        self.grid.nodes[current].frontier = False
        self.explored.append(current)
        children = [(current[0]+a[0], current[1]+a[1]) for a in ACTIONS]
        # loop through the children
        for node in children:
            # do not explore if already checked
            if node in self.explored:
                print("explored before: ", node)
                continue
            # make sure node is within the grid to prevent going outside the grid
            if node[0] in range(self.grid.row_range) and node[1] in range(self.grid.col_range):
                # cannot traverse puddles
                if self.grid.nodes[node].puddle:
                    print("puddle at: ", node)
                else: 
                    # create a pair to keep track of cost and the node
                    node_pair = (current_pair[0] + self.grid.nodes[node].cost(), node)

                    # make sure the the node with the distance is not already in the frontier
                    # prevents duplicates
                    if node_pair in self.frontier:
                        print("already in frontier: ", node_pair)
                        continue
                    # set the previous node to form a path (the node traversed to reach this child)
                    self.previous[node] = current
                    if node == self.goal:
                        self.finished = True
                        return
                    else:
                        # add this current child to the frontier with a total cost (prev cost + next node cost)
                        heappush(self.frontier, node_pair)
                        # set the boolean to denote this child as in the frontier
                        self.grid.nodes[node].frontier = True
        

    '''
    Performs a step of the A* algorithm, which is similar to UCS, but also takes into account
    a heuristic value when choosing which node to explore next.
    '''
    def astar_step(self):
        #[Hint] you need to declare a heuristic function for Astar
        
        # Check to see if the frontier is not empty
        if not self.frontier:
            self.failed = True
            print("no path")
            return

        # Pop the node with the smallest cost from the heap (should be a pair (Cost + H, Node, Cost))
        current_tuple = heappop(self.frontier)
        current = current_tuple[1]
        print("current node: ", current)
        # set this node as "explored"
        self.grid.nodes[current].checked = True
        self.grid.nodes[current].frontier = False
        self.explored.append(current)
        children = [(current[0]+a[0], current[1]+a[1]) for a in ACTIONS]
        # loop through the children
        for node in children:
            # do not explore if already checked
            if node in self.explored:
                print("explored before: ", node)
                continue
            # make sure node is within the grid to prevent going outside the grid
            if node[0] in range(self.grid.row_range) and node[1] in range(self.grid.col_range):
                # create a tuple to keep track of total cost, cost, and the node
                curr_cost = current_tuple[2] + self.grid.nodes[node].cost()

                # this is the current cost + h-value
                new_total_cost = curr_cost + self.dist(node, self.goal)

                node_tuple = (new_total_cost, node, curr_cost)

                # make sure the the node with the distance is not already in the frontier
                # prevents duplicates
                skip = False

                # create a list of pairs using just the node and the ACTUAL cost
                no_h_frontier = [(y,z) for x,y,z in self.frontier]
                for b,c in no_h_frontier:
                    # skip this child if there is a better way to get to it in the frontier
                    if node_tuple[1] == b and node_tuple[2] >= c:
                        print("worse than node in frontier: ", node_tuple)
                        skip = True
                        break
                if skip:
                    continue
                # cannot traverse puddles
                if self.grid.nodes[node].puddle:
                    print("puddle at: ", node)
                else: 
                    # set the previous node to form a path (the node traversed to reach this child)
                    self.previous[node] = current
                    if node == self.goal:
                        self.finished = True
                        return
                    else:
                        # add this current child to the frontier with a total cost (prev cost + next node cost)
                        heappush(self.frontier, node_tuple)
                        # set the boolean to denote this child as in the frontier
                        self.grid.nodes[node].frontier = True

    '''
    Calculate the heuristic between two nodes
    '''
    def dist(self, node_1, node_2):
        #return (abs(node_1[0] - node_2[0]) + abs(node_1[1] - node_2[1]))
        return ((node_1[0] - node_2[0])**2) + ((node_1[1] - node_2[1])**2)
        
