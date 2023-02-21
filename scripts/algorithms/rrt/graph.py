from copy import deepcopy
import cv2
import math
import matplotlib.pyplot as plt
import numpy as np
import yaml


class Node():
    def __init__(self, index = (-1, -1), f_score=math.inf, g_score=math.inf):
        self._position = (None, None )


        self._index = index

        self._x = self._position[0]
        self._y = self._position[1]

        self._f = f_score
        self._g = g_score

    def get_position(self):
        return self._position

class Edge:

    def __init__(self, weight):

        self._weight = weight
        
    def get_weight(self):
        return self._weight

class Graph:

    def __init__(self):
        self._graph_dict = dict() 
        self._idx_2_node = dict() 

    def add_node(self, node):
        if not self.contains(node):
            self._graph_dict[node] = dict() 

            self._idx_2_node[node._index] = node 

    def add_edge(self, node1, node2, weight):

        if node1 in self._graph_dict.keys() and node2 in self._graph_dict.keys():
            self._graph_dict[node1][node2] = Edge(weight)
            self._graph_dict[node2][node1] = Edge(weight)

        else:
            raise KeyError("Cannot Add an Edge between nodes that do not exist")

    def get_neighbors(self, node):
        node_ls = [] 

        neighbor_dict = self._graph_dict[node]

        for node, edge in neighbor_dict.items():

            node_ls.append(node)

        return node_ls


    def get_edge(self, node1, node2):
        for node , edge in self._graph_dict[node1].items():

            if node == node2:
                return edge.get_weight()

    def graph_dict(self):
        return self._graph_dict


    def get_node_from_index(self, idx):
        return self._idx_2_node[idx]

    def prune_graph(self):

        del_list = [] 

        for node, neighbors in self._graph_dict.items():

            if len(neighbors.keys()) ==0:
                del_list.append(node)

        for node in del_list:
            del self._graph_dict[node]


    def contains(self, node):
        return True if node in self._graph_dict.keys() else False
    

    def print_graph(self):
        for node, adj_dict in self._graph_dict.items():

            name_ls = [node._index for node, edge in adj_dict.items()]
            print(f"Node = {node._index}, Adjency List = {name_ls}")

class GridMapGraphGenerator:

    def_map = f'maps/map.pgm'
    def_conf = f'maps/map.yaml'

    def __init__(self, map=None, config=None, connected_8=True, inflate=True, map_high_thresh=254, major_diam=18, minor_diam=6):
        self._map_high = map_high_thresh
        self._original_map = None
        self._connectivity = connected_8
        
        # Parse Init parameters
        self._config = self._parse_config(config) if config else self._parse_config(self.def_conf)
        self._pgm_map = None 

        # Robot Specific Params 
        self._major_diam_in = major_diam
        self._minor_diam_in = minor_diam

        # Constants 
        self._inch_2_meters = 0.0254
        self._meters_2_inch = 39.3701
        self._minor_robot_diam = self._minor_diam_in * self._inch_2_meters        # Meters
        self._major_robot_diam =  self._major_diam_in * self._inch_2_meters       # Meters 

        # Map Params 
        self._resolution = self._config["resolution"] # meters/pixel
        self._mode = self._config["mode"]
        self._origin = self._config["origin"]
        self._occupied_thresh = self._config["occupied_thresh"]
        self._free_thresh = self._config["free_thresh"]
        self._pgm_name = self._config["image"]
        self._inflation_radius = 7# * (1 / self._resolution) # in Pixels

        # Apply Obstacle Inflation
        self._pgm_map = self._parse_map(map) if map else self._parse_map(self.def_map) 
        self._original_map = deepcopy(self._pgm_map)

        if inflate:
            self._pgm_map = None             
            self._pgm_map = self._inflate_map_obstacle(self._inflation_radius)

            # plt.imshow(my_first_map)
            # plt.show()


        # Create Empty Graph Data Structure
        self._graph = Graph()
        
    #############################
    #       Private Methods 
    #############################
    def _parse_map(self, file_name):
        image = cv2.imread(file_name,-1)
        return image 

    def _parse_config(self, conf_path):
        with open(conf_path, 'r') as file:
            conf = yaml.safe_load(file)
        return conf

    def _circle_check(self, center, point, radius):
        dist = math.sqrt((point[0] - center[0])**2 + (point[1] - center[1])**2)
        return True if dist < radius else False

    def _inflation_region(self, ind, radius, grid_size):
        buffer = 0

        grid_x, grid_y = grid_size[0]-1, grid_size[1]-1

        x, y = ind[0], ind[1]

        min_x_offset = int(x - radius - buffer) 
        min_y_offset = int(y - radius - buffer )
        max_x_offset = int(x + radius + buffer )
        max_y_offset = int(y + radius + buffer )

        min_x_ind = min_x_offset if min_x_offset > 0 else 0
        min_y_ind = min_y_offset if min_y_offset > 0 else 0
        max_x_ind = max_x_offset if max_x_offset < grid_x else grid_x
        max_y_ind = max_y_offset if max_y_offset < grid_y else grid_y

        inf_x_range = range(min_x_ind, max_x_ind)
        inf_y_range = range(min_y_ind, max_y_ind)

        return inf_x_range, inf_y_range 

    def _inflate_map_obstacle(self, obs_radius):
        inf_map = deepcopy(self._original_map)

        for center, val in np.ndenumerate(self._original_map): 

            if val == 0: 
                x_range, y_range = self._inflation_region(center, obs_radius, inf_map.shape)
                for x_ind in x_range:
                    for y_ind in y_range:
                        check = self._circle_check(center, (x_ind, y_ind), obs_radius)

                        if check:
                            inf_map[(x_ind, y_ind)] = 0
        return inf_map

    #############################
    #       Public Methods 
    #############################

    def reconstuct_and_plot_grid_map(self, graph=None):

        plt.imshow(self._pgm_map)
        plt.show()

        # if not graph:
        #     graph = self.build_graph()

        # mat = np.ndarray((self._pgm_map.shape[0], self._pgm_map.shape[1]))

        # for idx, val in np.ndenumerate(mat):
        #     node = graph.get_node_from_index(idx)
        #     mat[node._index] = 1 if node in graph.graph_dict().keys() else 0


        # plt.imshow(mat)
        # plt.show()

    def build_graph(self, prune=True):

        for idx, value in np.ndenumerate(self._pgm_map):
            self._graph.add_node(Node(index=(idx[0],idx[1])))

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


if __name__ == "__main__":

    map_path = f'map.pgm'
    # map_path = "20x20_map.pgm"
    conf_path = f'map.yaml'

    # graph_gen = GridMapGraphGenerator(map_path, conf_path, inflate=True)
    graph_gen = GridMapGraphGenerator( inflate=True)
    graph_gen.reconstuct_and_plot_grid_map()
    # pgm_graph = graph_gen.build_graph()
    # graph_gen.reconstuct_and_plot_grid_map(pgm_graph)
