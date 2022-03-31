import cv2
import numpy as np
from copy import deepcopy

from perception.oak.utils import *
from perception.oak.AR_tags import *


"""
TODO:
    - Add function for finding connected components (CCs) in the occupancy grid
    - Add function for computing centroid of each CC and assigning a weight to them by area
    - Add function for filtering out small, sparse CCs that are not close to the centroid of the largeset CC
    - Integrate these functions into a main mapping function that outputs an occupancy grid
"""


class OccMap(object):
    """
    Main class for creating and storing occupancy map.  May be beneficial to store as a graph or as
    a dict with set (for free space locations) and dict (for occupied space locations mapped to certainty or density)
    """
    def __init__(self, threshes, resolution=0.1, rows=50, cols=25):
        self.__pixel_d_thresh = threshes["distance"]
        self.__occ_rho_thresh = threshes["density"]
        self.__resolution = resolution
        self.__rows = rows / resolution
        self.__cols = cols / resolution
        self.__map = None
        self.__ground_plane = None

    def find_occ(self, masked_im, ground_plane):
        """
        Function which takes an image with pingpong balls emphasized (thresholded, segmented)
        and creates a occupancy grid in the AR World 2D space.
        
        """
        if self.__ground_plane is None:
            self.__ground_plane = deepcopy(ground_plane)
            self.__rows = len(ground_plane)
            self.__cols = len(ground_plane[0])
        
        occupied = {}
        free = set()
        n, m = len(ground_plane), len(ground_plane[0])
        for i in range(n):
            for j in range(m):
                x, y = ground_plane[i][j]
                if x > 0 and y > 0:
                    window = masked_im[y-self.__pixel_d_thresh:y+self.__pixel_d_thresh+1, x-self.__pixel_d_thresh:x+self.__pixel_d_thresh+1, :]
                    truth_mask = window > 0
                    if np.any(truth_mask):
                        density = np.sum(truth_mask) / (3*((2*self.__pixel_d_thresh)**2)) # store density
                        if density > self.__occ_rho_thresh:
                            occupied[(i, j)] = density
                    else:
                        free.add((i, j))
        mapped = {"occupied": occupied,
                  "free": free}
        self.__map = deepcopy(mapped)
        return mapped;
