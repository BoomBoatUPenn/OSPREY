import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from math import cos

# design class/functions for using pure pursuit to follow generated path
# order path nodes by distance from the current pose of the boat (L2 distance)
# choose the furthest path node within a particular radius and drive straight towards it

ROOT = ".\\planning\\preplanned\\"

class PurePursuit():
    def __init__(self, r=0.5, sim=True, preplanned=None):
        self.__r = r
        self.__sim = sim
        if preplanned is not None:
            with open(ROOT + preplanned, 'r', newline='\n') as f:
                path_nodes = f.readlines() #, dtype=np.float32)
                self.__path_nodes = np.empty((len(path_nodes), 2))
                for i, node in enumerate(path_nodes):
                    delim = node.index(',')
                    self.__path_nodes[i, :] = np.array([float(node[:delim]), float(node[delim+1:])])
        else:
            with open(ROOT + "line.txt", 'r', newline='\n') as f:
                path_nodes = f.readlines() #, dtype=np.float32)
                self.__path_nodes = np.empty((len(path_nodes), 2))
                for i, node in enumerate(path_nodes):
                    delim = node.index(',')
                    self.__path_nodes[i, :] = np.array([float(node[:delim]), float(node[delim+1:])])
        self.__current = 0
        self.__next = None
        self.__map = None
        self.__world_origin = None
        self.__speed = 0.5

    def find_curr_next(self, boat_pose):
        """
        Given the current boat pose, find the current node and the furthest, forward node within a search radius
        """
        # create array of distances between path nodes and the current pose
        dist_arr = np.linalg.norm(boat_pose - self.__path_nodes, axis=1)
        # set the current path node to be the one closest to the current pose
        current = np.argmin(dist_arr)
        # find indices of path nodes within radius r
        restricted_inds = np.argwhere(dist_arr <= self.__r)
        # find inds which are greater than the current index in the path
        ahead_inds = restricted_inds[restricted_inds > self.__current]
        # greedily choose the furthest, forward node within the lookahead radius
        next = max(ahead_inds)
        self.__current = current
        self.__next = next
        # return current, next;

    def pursue(self, boat_pose):
        """
        Given the current boat pose, define the steering angle and velocity
        """
        if self.__next is not None:
            next_pose = self.__path_nodes[self.__next]
            theta, pose = boat_pose
            y = abs((next_pose[1] - pose[1]) / cos(theta))
            curvature = 2.*y / self.__r
            return (curvature, self.__speed)
        return (0, self.__speed)


if __name__ == "__main__":
    r = 0.5
    p_pursuit_node = PurePursuit(r)
    boat_pose_i = np.array([0., 0.], dtype=np.float32)
    curr, next = p_pursuit_node.find_curr_next(boat_pose_i)
    speed = np.array([0, 1.0], dtype=np.float32)
    time_step = 0.2

    for i in range(10):
        # this can be updated using the perception code which extracts the boat pose relative to the world
        boat_pose = boat_pose_i + i*time_step*speed
        curr, next = p_pursuit_node.find_curr_next(boat_pose)
        print("curr", curr)
        print("next", next)
        print()
