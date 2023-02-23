import numpy as np 
import matplotlib.pyplot as plt 
from datastructures.graph import Graph
from datastructures.node import Node
from generators.graph_generator_base import GraphGeneratorBase

class RRTGraphGenerator(GraphGeneratorBase):

    def __init__(self, map=None, config=None, graph=Graph, node=Node, connected_8=True, inflate=True, map_high_thresh=254, major_diam=18, minor_diam=6):
        super().__init__(map, config, graph, node, connected_8, inflate, map_high_thresh, major_diam, minor_diam)

    def reconstuct_and_plot_grid_map(self, graph=None):
        pass

    def build_graph(self, prune=True):
        pass
