# ros2-py-path-planning-server 
This repo implements a ROS2 server for computing the path planning required for the [Future Mobility Lab: Autonomous Vehicle Project](https://github.com/GeorgeTMartin/FML_AutonomousCar)


## Path Planner Utility

```
cd ros2_py_path_planning_server
python3 planner.py --config /<path-to-config-yaml> --preview-path
```


## Planners 

- [x] A* 
- [ ] RRT (IN PROGRESS)
- [ ] RRT*
- [ ] PRM

## TODO List:

- [x] Create Graph from occupancy map map
- [x] Implement A* Proof of Concept 
- [x] Create ROS2 server/client Proof of Concept
- [ ] Refactor ROS2 Path service to create solve A* (IN PROGRESS)
- [x] Refactor A* implementation with better OOP fundamentals 
- [ ] Solve RRT/RRT*/PRM via OOP implementations 
- [ ] Add RRT/RRT*/PRM algorithm support to Path service
