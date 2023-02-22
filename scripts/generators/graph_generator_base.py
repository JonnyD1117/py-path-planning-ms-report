import math
import numpy as np 
import matplotlib.pyplot as plt 
from copy import deepcopy
import cv2
import yaml
from ..datastructures.graph import Graph  
from ..datastructures.node import Node
from abc import ABC, abstractmethod

class GraphGeneratorBase(ABC):

    def_map = f'maps/map.pgm'
    def_conf = f'maps/map.yaml'

    def __init__(self, map=None, config=None, graph=None, node=None, connected_8=True, inflate=True, map_high_thresh=254, major_diam=18, minor_diam=6):
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

        # Create Empty Graph Data Structure
        self._graph = graph()
        self._node = node
        
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
    #       Abstract Methods 
    #############################

    @abstractmethod
    def reconstuct_and_plot_grid_map(self, graph=None):
        pass

    @abstractmethod
    def build_graph(self, prune=True):
        pass