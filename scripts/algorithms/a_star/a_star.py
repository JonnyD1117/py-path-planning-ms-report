from graph import GridMapGraphGenerator
import math
import matplotlib.pyplot as plt
import numpy as np

from path_smoothing import BezierPathSmoothing


class AStar:
    def __init__(self, grid_map):
        self._graph = grid_map
        self._open_set = set()
        self._closed_set = set()
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

    def construct_shortest_path(self, current):
        total_path = [current]
        total_path_idx = [] 

        while current in self._prev_node.keys():
                prev = self._prev_node[current]
                total_path.append( prev)
                total_path_idx.append(prev._index)
                current = prev
        return total_path, total_path_idx

    def get_lowest_f_score_in_open_set(self):

        min_val = math.inf
        min_node = None
         
        for node in self._open_set:

            f_score = self._f_score[node]

            if f_score < min_val:
                min_val = f_score
                min_node = node

        return min_val, min_node
    
    def run(self, start_idx, end_idx):
        # Initialize Start and Goal Nodes
        start_node = self._graph.get_node_from_index(start_idx)
        end_node = self._graph.get_node_from_index(end_idx)

        # Initialize Node & Heuristic Costs
        self._f_score[start_node] = 0
        self._g_score[start_node] = 0

        # Initialize Open Set
        self._open_set.add(start_node)

        while self._open_set:
            f_score, current = self.get_lowest_f_score_in_open_set()

            if current == end_node:
                return self.construct_shortest_path(end_node)

            self._open_set.remove(current)
            self._closed_set.add(current)

            for neighbor in self._graph.get_neighbors(current):
                
                if neighbor in self._closed_set:
                    continue

                tentative_g_score = self._g_score[current] + self._graph.get_edge(current, neighbor) 

                if neighbor not in self._open_set:
                    self._open_set.add(neighbor)

                elif tentative_g_score >= self._g_score[neighbor]:
                    continue

                self._g_score[neighbor] = tentative_g_score
                self._f_score[neighbor] = self._g_score[neighbor] + self.heuristic(neighbor, end_node)
                self._prev_node[neighbor] = current                                                               

        return False

if __name__ == "__main__":

    map_path = "20x20_map.pgm"
    
    graph_gen = GridMapGraphGenerator(inflate=True, connected_8=False)
    mat = graph_gen._pgm_map
    non_inf_mat = graph_gen._original_map

    graph = graph_gen.build_graph()

    start_idx = (215, 230)
    goal_idx = (150, 170)

    # start_idx = (17, 6)
    # goal_idx = (17, 17)

    algo = AStar(grid_map=graph)
    output, output_idx = algo.run(start_idx, goal_idx)  


    b = BezierPathSmoothing(ctr_points=output_idx)
    x_bez, y_bez = b.compute_smooth_path()  


    if output:
        out_indx = [node._index for node in output]
        for idx, val in np.ndenumerate(mat):
            if idx in out_indx:
                mat[idx] = .5

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(y_bez[0], x_bez[0])


    x = [val[0] for val in output_idx]
    y = [val[1] for val in output_idx]

    # ax.scatter(x, y, c='black')
    plt.imshow(non_inf_mat)
    plt.show()

    # print(x_bez)

    plt.plot( y_bez[0], -x_bez[0])
    plt.show()
