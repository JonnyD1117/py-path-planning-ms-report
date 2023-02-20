#!/usr/bin/env python3
from ros2_path_planning_interfaces.srv import PathPlanner

import rclpy
from rclpy.node import Node

class PathPlannerService(Node):

    def __init__(self):
        super().__init__('path_planning_service')
        self.srv = self.create_service(PathPlanner, 'path_planner_service', self.compute_path)
    
    def compute_path(self, request, response):
        map_path = request.map_path
        response.path_json_path = "www.amazon.com"
        output_path = response.path_json_path
        self.get_logger().info(f"Map Exists at: {map_path}, & Output Exists at:{output_path}")
        return response
    
def main():
    rclpy.init()
    path_serv = PathPlannerService()
    rclpy.spin(path_serv)
    rclpy.shutdown()

if __name__ == "__main__":
    main()