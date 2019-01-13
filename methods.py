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
    def new_plan(self, type):
        self.finished = False
        self.failed = False
        self.type = type
        if self.type == "dfs" :
            self.frontier = [self.start]
            self.explored = []
        elif self.type == "bfs":
            pass
        elif self.type == "ucs":
            pass
        elif self.type == "astar":
            pass
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
    def dfs_step(self):
        #...
        if not self.frontier:
            self.failed = True
            print("no path")
            return
        current = self.frontier.pop()
        print("current node: ", current)
        #...
        self.grid.nodes[current].checked = True
        self.grid.nodes[current].frontier = False
        self.explored.append(current)
        children = [(current[0]+a[0], current[1]+a[1]) for a in ACTIONS]
        #...
        for node in children:
            #See what happens if you disable this check here
            if node in self.explored or node in self.frontier:
                print("explored before: ", node)
                continue
            #...
            if node[0] in range(self.grid.row_range) and node[1] in range(self.grid.col_range):
                #...
                if self.grid.nodes[node].puddle:
                    print("puddle at: ", node)
                else:
                    #...
                    self.previous[node] = current
                    if node == self.goal:
                        self.finished = True
                        return
                    else:
                        #...
                        self.frontier.append(node)
                        #...
                        self.grid.nodes[node].frontier = True
            else:
                print("out of range: ", node)
    def bfs_step(self):
        pass
    def ucs_step(self):
        #[Hint] you can get the cost of a node by node.cost()
        pass
    def astar_step(self):
        #[Hint] you need to declare a heuristic function for Astar
        pass
