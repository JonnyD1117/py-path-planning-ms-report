#!/usr/bin/env python3

from ros2_path_planning_interfaces.srv import PathPlanner
from scripts.a_star import AStar
from scripts.graph import GridMapGraphGenerator
from scripts.path_smoothing import BezierPathSmoothing
import json
from datetime import datetime
import os

import rclpy
from rclpy.node import Node

class PathPlannerService(Node):

    def __init__(self):
        super().__init__('path_planning_service')
        self.srv = self.create_service(PathPlanner, 'path_planner_service', self.compute_path)
        self._logger = self.get_logger()
        self._logger.info(f"Starting path planner service.")
    
    def compute_path(self, request, response):
        # Parse Input Parameters
        #self._map_path = request.map_path
        # json_path = request.json_path
        # self.planner_type = request.planner
        # start = (request.start_x, request.start_y)
        # goal = (request.goal_x, request.goal_y)

        # if planner_type == "a_star":
        #     self._solve_astar(map_path="src/ros2_py_path_planning_server/maps/map.pgm", start=(215, 230), goal=(150, 170))
        # elif planner_type == "rrt":
        #     pass
        # elif planner_type == "rrt*":
        #     pass
        # elif planner_type == "prm":
        #     pass
        # else:
        #     # Run A*
        #     pass

        # Parse Output Parameters
        response.path_json_path = "www.amazon.com"
        output_path = response.path_json_path
        self._logger.info(f"Map Exists at: {self.map_path}, & Output Exists at:{output_path}")
        return response

    def _solve_astar(self, map_path="src/ros2_py_path_planning_server/maps/map.pgm", start=(215, 230), goal=(150, 170)):

        # def _build_graph(map_path):
        #     """Generate graph from grip map."""
        #     graph_gen = GridMapGraphGenerator(inflate=True, connected_8=False)
        #     return graph_gen.build_graph(map_path)

        # def _build_json(raw_wpts, smooth_wpts, planner, path):
        #     """Generate the output JSON object and save to a JSON file."""

        #     dt_str = datetime.now().strftime("") 
        #     output_name = f"{dt_str}_{planner}.json"
        #     json_path = os.path.join(output_name, path)

        #     path_planner_dict = {
        #         "Planner": planner,
        #         "map": None, 
        #         "timestamp" : dt_str,
        #         "raw_wpts": raw_wpts,
        #         "smooth_wpts": smooth_wpts
        #     }

        #     with open(json_path, 'w') as json_file:
        #         json.dump(path_planner_dict, json_file)

        #     return json_path


        # # Create Configuration Graph   
        # _graph = _build_graph()

        # # Solve A* over Configuration Graph
        # algo = AStar(grid_map=_graph)
        # output = algo.run(start, goal)  

        # if output:
        #     raw_wpts  = output[1]
        # else:
        #     self._logger.error(f"A* could NOT find a feasiable path from {start} -> {output}")


        # # Apply Path Smoothing to output Waypoints
        # x_bez, y_bez = BezierPathSmoothing(ctr_points=raw_wpts).compute_smooth_path() 

        # # Build and Export output file
        # output_path = _build_json(raw_wpts, smooth_wpts, self.planner_type, path)

        # return output

        pass

    def _solve_rrt(self, start, goal):
        pass

    def _solve_rrtstar(self,start, goal):
        pass

    def _solve_prm(self,start, goal):
        pass

    
def main():
    rclpy.init()
    path_serv = PathPlannerService()
    rclpy.spin(path_serv)
    rclpy.shutdown()

if __name__ == "__main__":
    main()