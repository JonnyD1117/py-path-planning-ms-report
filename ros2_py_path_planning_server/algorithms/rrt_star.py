from graph import Graph, Node, Edge
import numpy as np 
import random

class RRT:
    def __init__(self, map, start_pos, goal_pos, num_nodes, delta_pos):
        self._graph = Graph()

        self._map = map
        self._map_size = map.shape

        self._start_pos = start_pos
        self._goal_pos = goal_pos 
        self._num_nodes = num_nodes
        self._delta_pos = delta_pos

        self._graph.add_node(Node(index=self._start_pos))

    def _is_obstacle(self, pos):

        if self._map[pos] == 0:
            return True
        else:
            return False

    def _sample_random_configuration(self):

        is_free = False

        while not is_free:
            x_rand = random.uniform(0, self._map_size[1])
            y_rand = random.uniform(0, self._map_size[0])

            rand_pos = (x_rand, y_rand)
            is_free = self._is_obstacle(rand_pos)

    def _nearest_node(self, pos):
        pass

    def _compute_step(self, cur_node, new_node):

        cur_pos = cur_node.get_index()
        new_pos = new_node.get_index()

        #

    def run(self):

        for k in range(self._num_nodes):
            pass





    



if __name__ == '__main__':

    map_path = f"/home/indy/repos/ros2-py-path-planning-server/maps/20x20_map.pgm"

    graph = Graph()

    start_pos = (17, 6)
    goal_pos = (18, 17)

    init_node = Node(index=start_pos)

    delta_pos = .01
    num_nodes = 100