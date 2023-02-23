#!/usr/bin/env python3
from argparse import ArgumentParser
from datetime import datetime
import json
from pathlib import Path
import yaml
import pprint 
import matplotlib.pyplot as plt 
import numpy as np


from algorithms.a_star import AStar
from post_processing.path_smoothing import BezierPathSmoothing
from generators.a_star_graph_generator import AstarGraphGenerator


pp = pprint.PrettyPrinter(indent=4)

conf_path = Path("configs/default_config.yaml").absolute()

parser = ArgumentParser()
parser.add_argument(
    "--config",
    default=str(conf_path),
    help="Path to a Path Planner yaml configuration file."
)
parser.add_argument(
    "--preview-path",
    default=False,
    action="store_true",
    help="Option to preview the solved path if it exists"
)

def main():

    args = parser.parse_args()
    with open(args.config , 'r') as file:
        config = yaml.safe_load(file)["FML"]
    
    algo = AStar(config)

    mat = algo._graph_generator._pgm_map
    non_inf_mat = algo._graph_generator._original_map

    output, output_idx = algo.run()

    b = BezierPathSmoothing(ctr_points=output_idx)
    x_bez, y_bez = b.compute_smooth_path()  

    x_smooth = x_bez.tolist()[0]
    y_smooth = y_bez.tolist()[0]


    output_smooth = [(x_smooth[i], y_smooth[i]) for i in range(len(x_smooth))]



    json_dict = {"raw_wpts": output_idx, "smoothed_wpts": output_smooth}

    dt = datetime.now().strftime("%Y%m%dT%H%M%S")

    file_name = f"{dt}_path_planner.json"


    json_path = f"{config['output']['path']}/{file_name}" 
    
    with open(json_path, "w") as outfile:
        json.dump(json_dict, outfile)

    if args.preview_path:
        fig = plt.figure()
        ax1 = fig.add_subplot(1,2,1)
        ax1.plot(y_bez[0], x_bez[0])
        ax1.imshow(mat)
        ax2 = fig.add_subplot(1,2,2)
        ax2.plot(y_bez[0], x_bez[0])
        ax2.imshow(non_inf_mat)
        plt.show()


if __name__ == "__main__":
    main()    