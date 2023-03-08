#!/bin/env python3 
import matplotlib.pyplot as plt 
import numpy as np 
import json 

if __name__ == "__main__":
    path = "/home/indy/temp/python_plot/lidar_test_scan/hallway_threeway_junc.json"
    # path = "/home/indy/temp/python_plot/lidar_test_scan/hallway_fml.json"
    # path = "/home/indy/temp/python_plot/lidar_test_scan/elevator_hallway.json"

    # path = "/home/indy/temp/python_plot/lidar_data_new.json"



    f = open(path)

    data = json.load(f)
    print(data.keys())

    # Angles and Increments are in radians
    start_angle = data["angle_min"]
    end_angle = data['angle_max']
    angle_increment = data["angle_increment"]

    ranges = data["ranges"]

    min_range = data["range_min"]
    max_range = data['range_max']

    filt_ranges = [] 

    for dist in ranges:

        if dist > max_range or dist < min_range:
            dist = 0

        filt_ranges.append(dist)
         



    theta = np.arange(start_angle, end_angle, angle_increment)

    fig = plt.figure()
    ax = fig.add_subplot(projection='polar')
    c = ax.scatter(theta, filt_ranges, cmap='hsv', alpha=0.75)



    # fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    # ax.plot(theta, filt_ranges)
    # ax.set_rmax(2)
    # ax.set_rticks([0.5, 1, 1.5, 2])  # Less radial ticks
    # ax.set_rlabel_position(-22.5)  # Move radial labels away from plotted line
    ax.grid(True)

    # ax.set_title("A line plot on a polar axis", va='bottom')
    plt.show()

