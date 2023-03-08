import math
from node import Node
from queue import PriorityQueue
import numpy as np

from graph import create_gpm_graph

import matplotlib.pyplot as plt



class AStar:
    def __init__(self, grid_map):
        self._graph = grid_map
        self._open_queue = PriorityQueue()

        self._prev_node = dict()

        self._f_score = {node: math.inf for node in self._graph.graph_dict().keys()}
        self._g_score = {node: math.inf for node in self._graph.graph_dict().keys()}


    def heuristic(self, curr_node, goal_node, discount=.95):

        x_start = curr_node._index[1]
        x_goal = goal_node._index[1]

        y_start = curr_node._index[0]
        y_goal = goal_node._index[0]

        heur_dist = math.sqrt((x_goal - x_start)**2 + (y_goal-y_start)**2)

        # return abs(x_start - x_goal) + abs(y_start - y_goal)

        return heur_dist * discount


    def find_closest_node(self, idx):
        # Return the nearest position
        return self._graph.get_node_from_index(idx)

    def construct_shortest_path(self, current):
        total_path = [current]

        while current in self._prev_node.keys():
                prev = self._prev_node[current]
                total_path.append( prev)
                current = prev
        return total_path


    def exists_open_queue(self, item):
        sorted_list = [] 
        while self._open_queue.empty()  == False:
            sorted_list.append(self._open_queue.get()[1])
        return item in sorted_list

    
    def run(self, start_idx, end_idx):
        # Initialize Start and Goal Nodes
        start_node = self._graph._idx_2_node[start_idx]
        end_node = self._graph._idx_2_node[end_idx]

        # Initialize Node & Heuristic Costs
        self._f_score[start_node] = self.heuristic(start_node, end_node)
        self._g_score[start_node] = 0
        self._open_queue.put((self._f_score[start_node], start_node))


        while not self._open_queue.empty():
            # Priority Queue will always be sorted by lowest -> highest f_score.
            current = self._open_queue.get()

            if current[1] == end_node:
                return self.construct_shortest_path(end_node)
                
            for neighbor in self._graph.get_neighbors(current[1]):

                tentative_g_score = self._g_score[current[1]] + self._graph.get_edge_weight(current[1], neighbor)

                if tentative_g_score < self._g_score[neighbor]:
                    self._prev_node[neighbor] = current[1]
                    self._g_score[neighbor] = tentative_g_score
                    self._f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, end_node)
                    
                    if not self.exists_open_queue(neighbor):
                        self._open_queue.put((self._f_score[neighbor], neighbor))

        return False

if __name__ == "__main__":
    
    graph,mat = create_gpm_graph()

    start_idx = (220, 200)
    goal_idx = (215, 234) #(212, 212)

    # start_idx = (150, 200)
    # goal_idx = (148, 234) #(212, 212)

    algo = AStar(grid_map=graph)
    output = algo.run(start_idx, goal_idx)    
    print(output)


    if output:

        out_indx = [node._index for node in output]

        for idx, val in np.ndenumerate(mat):

            if idx in out_indx:
                mat[idx] = .5



        
        # print(out_indx)

        plt.imshow(mat)
        plt.show()

    # print(algo._prev_node)
