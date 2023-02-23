import cv2
import numpy as np
import matplotlib.pyplot as plt

def erode_img(img, kernel_shape=(3,3), num_iter=4):
    kernel = np.ones(kernel_shape, np.uint8)
    img_erosion = cv2.erode(img, kernel, iterations=num_iter)
    return img_erosion

def dilate_img(img, kernel_shape=(3,3), num_iter=5):
    kernel = np.ones(kernel_shape, np.uint8)
    img_dilation = cv2.dilate(img, kernel, iterations=num_iter)
    return img_dilation

if __name__ == "__main__":
    img = cv2.imread('/home/indy/repos/ros2-py-path-planning-server/ros2_py_path_planning_server/maps/map_1677110999.pgm', 0)
  
    kernel = np.ones((3, 3), np.uint8)

    img_erosion = cv2.erode(img, kernel, iterations=4)
    img_dilation = cv2.dilate(img, kernel, iterations=5)
    
    composite_img = img_dilation

    for index , val in np.ndenumerate(img_erosion):

        if val ==0:
            composite_img[index] = val

    plt.imshow(composite_img)
    plt.show()
