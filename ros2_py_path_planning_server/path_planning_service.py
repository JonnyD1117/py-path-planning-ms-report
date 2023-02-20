#!/usr/bin/env python3
from ros2_path_planning_interfaces.srv import DummySrv

import rclpy
from rclpy.node import Node

class PathPlannerService(Node):

    def __init__(self):
        super().__init__('path_planning_service')
        self.get_logger().info(f"Maaaan FUck a diss server shit.")
        self.srv = self.create_service(DummySrv, 'add_two_ints', self.dummy_callback)

    def dummy_callback(self, request, response):
        response.sum = request.a + request.b 
        self.get_logger().info(f"{request.a} + {request.b} = {response.sum}")
        return response
    
def main():

    rclpy.init()
    path_serv = PathPlannerService()
    rclpy.spin(path_serv)
    rclpy.shutdown()

if __name__ == "__main__":
    main()