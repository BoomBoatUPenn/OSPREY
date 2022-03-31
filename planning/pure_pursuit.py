import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from math import cos

# design class/functions for using pure pursuit to follow generated path
# order path nodes by distance from the current pose of the boat (L2 distance)
# choose the furthest path node within a particular radius and drive straight towards it

ROOT = ".\\planning\\preplanned\\"

class PurePursuit():
    def __init__(self, speed_scale=0.3, r=0.5, sim=True, preplanned=None):
        self.__r = r
        self.__sim = sim
        if preplanned is not None:
            with open(ROOT + preplanned, 'r', newline='\n') as f:
                path_nodes = f.readlines() #, dtype=np.float32)
                self.__path_nodes = np.empty((len(path_nodes), 2))
                for i, node in enumerate(path_nodes):
                    delim = node.index(',')
                    self.__path_nodes[i, :] = np.array([float(node[delim+1:]), float(node[:delim])])
        else:
            with open(ROOT + "line.txt", 'r', newline='\n') as f:
                path_nodes = f.readlines() #, dtype=np.float32)
                self.__path_nodes = np.empty((len(path_nodes), 2))
                for i, node in enumerate(path_nodes):
                    delim = node.index(',')
                    self.__path_nodes[i, :] = np.array([float(node[delim+1:]), float(node[:delim])])
        self.__current = 0
        self.__next = None
        self.__speed = 0.0
        self.__high_speed = 1.0 # speed mapping
        self.__mid_speed = 2.0 / 3.0
        self.__low_speed = 1.0 / 3.0
        self.__speed_scale = speed_scale

    def find_curr_next(self, boat_pose):
        """
        Given the current boat pose, find the current node and the furthest, forward node within a search radius
        """
        boat_pose = np.array(boat_pose)
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
    
    def set_speed(self, alpha):
        """
        Take in the current steering angle and compute speed (mapped 0-1) for the boat
        """
        alpha_mag = abs(alpha)
        if alpha_mag >= 0:
            if alpha_mag < 0.33: # small steering angle -> scaled high speed
                speed = self.__speed_scale*self.__high_speed
            elif alpha_mag < 0.66: # middle steering angle -> scaled mid speed
                speed = self.__speed_scale*self.__mid_speed
            else: # large steering angle -> scaled low speed
                speed = self.__speed_scale*self.__low_speed
            self.__speed = speed
            return speed

    def pursue(self, boat_pose):
        """
        Given the current boat pose, define the steering angle and velocity
        """
        if self.__next is not None:
            next_pose = self.__path_nodes[self.__next]
            pose, theta = boat_pose
            y = (next_pose[1] - pose[1]) / cos(theta)
            curvature = 2.*y / self.__r
            curvature_clamped = min(1.0, max(-1.0, curvature))
            speed = self.set_speed(curvature_clamped)
            return (curvature_clamped, self.__speed)
        return (0.0, 0.5)


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
