import numpy as np 
import matplotlib.pyplot as plt 
from datastructures.graph import Graph
from datastructures.node import Node
from generators.graph_generator_base import GraphGeneratorBase

class AstarGraphGenerator(GraphGeneratorBase):

    def __init__(self, map=None, config=None, graph=Graph, node=Node, connected_8=True, inflate=True, map_high_thresh=254, major_diam=18, minor_diam=6):
        super().__init__(map, config, graph, node, connected_8, inflate, map_high_thresh, major_diam, minor_diam)

    def reconstuct_and_plot_grid_map(self, graph=None):

        # plt.imshow(self._pgm_map)
        # plt.show()

        # if not graph:
        #     graph = self.build_graph()

        # mat = np.ndarray((self._pgm_map.shape[0], self._pgm_map.shape[1]))

        # for idx, val in np.ndenumerate(mat):
        #     node = graph.get_node_from_index(idx)
        #     mat[node._index] = 1 if node in graph.graph_dict().keys() else 0


        # plt.imshow(mat)
        # plt.show()
        pass

    def build_graph(self, prune=True):

        for idx, value in np.ndenumerate(self._pgm_map):
            self._graph.add_node(self._node(index=(idx[0],idx[1])))

        # Create all Edges in the Graph (Connected-4)
        for idx, value in np.ndenumerate(self._pgm_map):

            if value < self._map_high:
                continue

            i, j = idx[0], idx[1]

            n_offset = i-1 if i-1 >= 0 else 0
            s_offset = i+1 if i+1 <self._pgm_map.shape[0] else self._pgm_map.shape[0] -1
            e_offset = j-1 if j-1 >= 0 else 0
            w_offset = j+1 if j+1 <self._pgm_map.shape[1] else self._pgm_map.shape[1] -1


            ne_idx = (n_offset, e_offset) 
            nw_idx = (n_offset, w_offset) 

            se_idx = (s_offset, e_offset) 
            sw_idx = (s_offset, w_offset)


            north_idx = (n_offset, j)
            south_idx = (s_offset, j)
            east_idx = (i, e_offset)
            west_idx = (i, w_offset)

            cur_node = self._graph.get_node_from_index(idx)

            # North Node 
            if self._pgm_map[north_idx] == self._map_high:
                north_node = self._graph.get_node_from_index(north_idx)
                self._graph.add_edge(cur_node, north_node, 1)
            # South Node
            if self._pgm_map[south_idx] == self._map_high:
                south_node = self._graph.get_node_from_index(south_idx)
                self._graph.add_edge(cur_node, south_node, 1)
            # West Node
            if self._pgm_map[west_idx] == self._map_high:
                west_node = self._graph.get_node_from_index(west_idx)
                self._graph.add_edge(cur_node, west_node, 1)
            # East Node
            if self._pgm_map[east_idx] == self._map_high:
                east_node = self._graph.get_node_from_index(east_idx)
                self._graph.add_edge(cur_node, east_node, 1)

            if self._connectivity:
                # North West Node 
                if self._pgm_map[nw_idx] == self._map_high:
                    nw_node = self._graph.get_node_from_index(nw_idx)
                    self._graph.add_edge(cur_node, nw_node, 1)
                # North East Node 
                if self._pgm_map[ne_idx] == self._map_high:
                    nw_node = self._graph.get_node_from_index(ne_idx)
                    self._graph.add_edge(cur_node, nw_node, 1)
                # South West Node 
                if self._pgm_map[sw_idx] == self._map_high:
                    sw_node = self._graph.get_node_from_index(sw_idx)
                    self._graph.add_edge(cur_node, sw_node, 1)
                # South East Node 
                if self._pgm_map[se_idx] == self._map_high:
                    se_node = self._graph.get_node_from_index(se_idx)
                    self._graph.add_edge(cur_node, se_node, 1)

        if prune:
            self._graph.prune_graph()

        return self._graph
