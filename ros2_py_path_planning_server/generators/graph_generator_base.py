from abc import ABC, abstractmethod
from copy import deepcopy
import cv2
import math
import numpy as np 
import yaml
from pre_processing.erosion_n_dilation import dilate_img, erode_img


class GraphGeneratorBase(ABC):
    default_config = "configs/default_config.yaml"

    def __init__(self, config=None, graph=None, node=None):
        self._graph = graph()
        self._node = node
        self._config = config if config else self.default_config

        # Extract Grid Map and Config Paths 
        self._map_path, self._map_conf_path = self._extract_map_path(config["map"]["path"])
        self._map_conf = self._parse_config(self._map_conf_path)
        self._pgm_map = self._parse_map(self._map_path)
        self._original_map = deepcopy(self._pgm_map)

        # General Configuration
        self._map_high = config["map"]["threshold"]
        self._connectivity = True if config["planner"]["connectivity"] == "connected-8" else False
        self._major_diam_in = config["vehicle"]["major_diameter"]
        self._minor_diam_in = config["vehicle"]["minor_diameter"]
        self._inflation_radius = self._config["map"]["inflation_radius"] # in Pixels

        # Map Configuration 
        self._resolution = self._map_conf["resolution"] # meters/pixel
        self._mode = self._map_conf["mode"]
        self._origin = self._map_conf["origin"]
        self._occupied_thresh = self._map_conf["occupied_thresh"]
        self._free_thresh = self._map_conf["free_thresh"]
        self._pgm_name = self._map_conf["image"]

        # Constants 
        self._inch_2_meters = 0.0254
        self._meters_2_inch = 39.3701
        self._minor_robot_diam = self._minor_diam_in * self._inch_2_meters        # Meters
        self._major_robot_diam =  self._major_diam_in * self._inch_2_meters       # Meters  

        input_map = self._original_map

        input_map = self._dilate_and_erode_map(input_map)

    
        # Apply Obstacle Inflation
        if self._config["map"]["inflate"]:            
            input_map = self._inflate_map_obstacle(self._inflation_radius, input_map)

        self._pgm_map = input_map 
        
        
    #############################
    #       Private Methods 
    #############################
    def _extract_map_path(self, config_path):
        root_path = config_path.split(".")[0]
        map_path = f"{root_path}.pgm"
        conf_path = f"{root_path}.yaml"
        return map_path, conf_path

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

    def _inflate_map_obstacle(self, obs_radius, input_map):
        inf_map = deepcopy(input_map)

        for center, val in np.ndenumerate(input_map): 

            if val == 0: 
                x_range, y_range = self._inflation_region(center, obs_radius, inf_map.shape)
                for x_ind in x_range:
                    for y_ind in y_range:
                        check = self._circle_check(center, (x_ind, y_ind), obs_radius)

                        if check:
                            inf_map[(x_ind, y_ind)] = 0
        return inf_map

    def _dilate_and_erode_map(self, input_map):

        if self._config["map"]["cv"]["apply"]:
        
            # Apply Erosion
            shape = self._config["map"]["cv"]["kernel"]
            iter = self._config["map"]["cv"]["erosion"]["num_iter"]
            eroded_map = erode_img(input_map, shape,iter)

            # Apply Dialation 
            shape = self._config["map"]["cv"]["kernel"]
            iter = self._config["map"]["cv"]["dilation"]["num_iter"]
            dilated_map = dilate_img(input_map, shape,iter)

            composite_img = dilated_map

            for index , val in np.ndenumerate(eroded_map):

                if val ==0:
                    composite_img[index] = val

            return composite_img
        return input_map


    #############################
    #       Abstract Methods 
    #############################

    @abstractmethod
    def reconstuct_and_plot_grid_map(self, graph=None):
        pass

    @abstractmethod
    def build_graph(self, prune=True):
        pass