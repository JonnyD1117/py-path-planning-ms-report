from copy import deepcopy
import cv2
from edge import Edge
import math
import matplotlib.pyplot as plt
from node import Node 
import numpy as np
import yaml


class Graph:

    def __init__(self):
        self._graph_dict = dict() 
        self._idx_2_node = dict() 

    def add_node(self, node):
        if not self._contains(node):
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


    def _contains(self, node):
        return True if node in self._graph_dict.keys() else False
    

    def print_graph(self):
        for node, adj_dict in self._graph_dict.items():

            name_ls = [node._index for node, edge in adj_dict.items()]
            print(f"Node = {node._name}, Adjency List = {name_ls}")


class GridMapGraphGenerator:

    def __init__(self, map, config, connected_4=True, inflate=True):
        # Parse Init parameters
        self._pgm_map = self._parse_map(map) 
        self._config = self._parse_config(config)
        self._connected_4 = connected_4

        # Robot Specific Params 
        self._major_diam_in = 18
        self._minor_diam_in = 6

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
        self._inflation_radius = self._minor_robot_diam * (1 / self._resolution) # in Pixels

        # Apply Obstacle Inflation
        if inflate:
            self._pgm_map = self._inflate_map_obstacle(self._inflation_radius, self._resolution)

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

    def _inflation_region(self, ind, pix_resolution, grid_size):

        buffer = 2

        grid_x, grid_y = grid_size[0], grid_size[1]

        x, y = ind[0], ind[1]

        min_x_offset = int(x - pix_resolution - buffer) 
        min_y_offset = int(y - pix_resolution - buffer )
        max_x_offset = int(x + pix_resolution + buffer )
        max_y_offset = int(y + pix_resolution + buffer )

        min_x_ind = min_x_offset if min_x_offset > 0 else 0
        min_y_ind = min_y_offset if min_y_offset > 0 else 0
        max_x_ind = max_x_offset if max_x_offset < grid_x else grid_y
        max_y_ind = max_y_offset if max_y_offset < grid_y else grid_x

        inf_x_range = range(min_x_ind, max_x_ind+1)
        inf_y_range = range(min_y_ind, max_y_ind+1)

        return inf_x_range, inf_y_range 

    def _inflate_map_obstacle(self, obs_radius, res):

        inf_map = deepcopy(self._pgm_map)

        for center, val in np.ndenumerate(map):
            if val == 0:

                x_range, y_range = self._inflation_region(center, res, map.shape)

                for x_ind in x_range:
                    for y_ind in y_range:

                        check = self._circle_check(center, (x_ind, y_ind), obs_radius)

                        if check:
                            inf_map[(x_ind, y_ind)] = val
            else:
                continue
        return inf_map

    #############################
    #       Public Methods 
    #############################


    def reconstuct_and_plot_grid_map(self, graph):

        mat = np.ndarray((self._pgm_map.shape[0], self._pgm_map.shape[1]))

        count = 0 
        for idx, val in np.ndenumerate(mat):

            node = graph.get_node_from_index(idx)

            # Test to Check the connectivity of the nodes in the graph 
            # if count % 25 == 0:
            #     if node in self._graph.graph_dict().keys():
            #         nd = self._graph.get_node_from_index(idx)
            #         neigbors = self._graph.get_neighbors(nd)
            #         # print(neigbors)
            #         for n in neigbors:
            #             n_idx = n._index
            #             mat[n_idx] = .5

            mat[node._index] = 1 if node in graph.graph_dict().keys() else 0

            count +=1 

        plt.imshow(mat)
        plt.show()



    def build_C4_graph(self):

        for idx, value in np.ndenumerate(self._pgm_map):
            self._graph.add_node(Node(index=(idx[0],idx[1]),name=f"row_{idx[0]}_col_{idx[1]}"))

        # Create all Edges in the Graph (Connected-4)
        for idx, value in np.ndenumerate(self._pgm_map):

            if value != 254:
                continue

            i, j = idx[0], idx[1]

            north_offset = i-1 if i-1 >= 0 else 0
            south_offset = i+1 if i+1 <self._pgm_map.shape[0] else self._pgm_map.shape[0] -1
            east_offset = j-1 if j-1 >= 0 else 0
            west_offset = j+1 if j+1 <self._pgm_map.shape[1] else self._pgm_map.shape[1] -1

            north_idx = (north_offset, j)
            south_idx = (south_offset, j)
            east_idx = (i, east_offset)
            west_idx = (i, west_offset)

            cur_node = self._graph.get_node_from_index(idx)


            if self._pgm_map[north_idx] == 254:
                north_node = self._graph.get_node_from_index(north_idx)
                self._graph.add_edge(cur_node, north_node, 1)


            if self._pgm_map[south_idx] == 254:
                south_node = self._graph.get_node_from_index(south_idx)
                self._graph.add_edge(cur_node, south_node, 1)

            if self._pgm_map[west_idx] == 254:
                west_node = self._graph.get_node_from_index(west_idx)
                self._graph.add_edge(cur_node, west_node, 1)


            if self._pgm_map[east_idx] == 254:
                east_node = self._graph.get_node_from_index(east_idx)
                self._graph.add_edge(cur_node, east_node, 1)

        self._graph.prune_graph()

        return self._graph



    def build_C8_graph(self):
        raise NotImplementedError("Connected 8 Graph has not been Implemented")



if __name__ == "__main__":

    map_path = f'map.pgm'
    conf_path = f'map.yaml'

    graph_gen = GridMapGraphGenerator(map_path, conf_path)

    pgm_graph = graph_gen.build_C4_graph()

    graph_gen.reconstuct_and_plot_grid_map(pgm_graph)

    
    