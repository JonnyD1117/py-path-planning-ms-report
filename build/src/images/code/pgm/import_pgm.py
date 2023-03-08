import cv2 
import numpy as np
from matplotlib import pyplot as plt



def import_gpm(file_name="/home/indy/map.pgm"):
	image = cv2.imread(file_name,-1)
	return image 
