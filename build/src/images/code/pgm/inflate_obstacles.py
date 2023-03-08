import numpy as np 
import matplotlib.pyplot as plt
from import_pgm import import_gpm
import math
from copy import deepcopy
import yaml

def circle_check(center, point, radius):
    dist = math.sqrt((point[0] - center[0])**2 + (point[1] - center[1])**2)

    if dist < radius:
        return True
    else:
        return False

def inflation_region(ind, pix_resolution, grid_size):

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

def inflate_map_obstacle(map, obs_radius, res):

    inf_map = deepcopy(map)

    for center, val in np.ndenumerate(map):
        if val == 0:

            x_range, y_range = inflation_region(center, res, map.shape)

            for x_ind in x_range:
                for y_ind in y_range:

                    check = circle_check(center, (x_ind, y_ind), obs_radius)

                    if check:
                        inf_map[(x_ind, y_ind)] = val
        else:
            continue
    return inf_map


if __name__ == "__main__":

    pgm_conf = "/home/indy/temp/pgm/map.yaml"

    with open(pgm_conf, 'r') as file:
        pgm_obj = yaml.safe_load(file)

    resolution = pgm_obj["resolution"] # meters/pixel
    mode = pgm_obj["mode"]
    origin = pgm_obj["origin"]
    occupied_thresh = pgm_obj["occupied_thresh"]
    free_thresh = pgm_obj["free_thresh"]
    pgm_name = pgm_obj["image"]

    inch_2_meters = 0.0254
    meters_2_inch = 39.3701
    minor_robot_diam = 6 * inch_2_meters        # Meters
    major_robot_diam = 18 * inch_2_meters       # Meters 

    inflation_radius = minor_robot_diam * (1 / resolution) # in Pixels


    pgm = import_gpm()
    inf_map = inflate_map_obstacle(pgm, inflation_radius, resolution)

    plt.imshow(pgm)
    plt.show()