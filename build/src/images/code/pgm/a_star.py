import math
from node import Node
from queue import PriorityQueue

from graph import create_gpm_graph



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

        return heur_dist * 1


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

        print(f"Start F = { self._f_score[start_node]}")

        self._open_queue.put((self._f_score[start_node], start_node))


        while not self._open_queue.empty():
            # Priority Queue will always be sorted by lowest -> highest f_score.
            current = self._open_queue.get()

            # breakpoint()

            if current[1] == end_node:

                print("Fuck me right")
                return self.construct_shortest_path(start_node)


            for neighbor in self._graph.get_neighbors(current[1]):

                tentative_g_score = self._g_score[current[1]] + self._graph.get_edge_weight(current[1], neighbor)

                if tentative_g_score < self._g_score[neighbor]:
                    self._prev_node[neighbor] = current[1]

                    self._g_score[neighbor] = tentative_g_score
                    self._f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, end_node)
                    
                    if not self.exists_open_queue(neighbor):
                        self._open_queue.put((self._f_score[neighbor], neighbor))
        
        # No Reachable Path to Goal
        return False

if __name__ == "__main__":
    
    graph = create_gpm_graph()

    start_idx = (220, 200)
    goal_idx = (212, 212)

    algo = AStar(grid_map=graph)
    output = algo.run(start_idx, goal_idx)    
    print(output)
    # print(algo._prev_node)
