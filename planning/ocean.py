import numpy as np
from numpy.random import random
from copy import deepcopy


class GridWorld(object):
    def __init__(self, n, *args, **kwargs):

        # initialize gridworld w/ oil spill in the center of fram
        self.n = n
        self.grid_world = np.zeros((self.n, self.n))
        self.offset = int(0.25*n)
        for i in range(self.offset - 1, self.n - self.offset + 1):
            i_thresh = (abs(int(self.n / 2) - i) / self.n)
            for j in range(self.offset - 1, self.n - self.offset + 1):
                j_thresh = (abs(int(self.n / 2) - j) / self.n)
                total_thresh = 0.85*(1.0 - 20*(i_thresh*j_thresh)) # a bit hacky
                if random() < total_thresh:
                    self.grid_world[i, j] = -50.0

        self.l_start = (self.n - int(0.5*self.offset), int((self.n - self.offset)/ 2))
        self.r_start = (self.n - int(0.5*self.offset), int((self.n + self.offset)/ 2))
        self.l_current = self.l_start
        self.r_current = self.r_start
        self.grid_world[self.l_current] = -51.0
        self.grid_world[self.r_current] = -50.0

        self.l_goal = (0,int((self.n - self.offset)/ 2))
        self.r_goal = (0, int((self.n + self.offset)/ 2))
        self.grid_world[self.l_goal] = 51.0
        self.grid_world[self.r_goal] = 50.0

        return;

    def get_neighbors(self, left, current=None):
        neighbors = []
        if current is not None:
            curr_i, curr_j = current
        elif left:
            curr_i, curr_j = self.l_current
        else:
            curr_i, curr_j = self.r_current
        
        for i in range(-1, 2):
            neighbor_i = curr_i + i
            for j in range(-1, 2):
                neigh = (neighbor_i, curr_j + j)
                if (neigh[0] >= 0 and neigh[0] < self.n) and (neigh[1] >= 0 and neigh[1] < self.n):
                    if self.grid_world[neigh] >= 0:
                        yield neigh;

    def make_move(self, l_move, r_move):
        l_d_i, l_d_j = l_move
        r_d_i, r_d_j = r_move

        l_curr_i, l_curr_j = self.l_current
        l_new_pos = (curr_i + d_i, curr_j + d_j)
        self.l_current = new_pos

        curr_i, curr_j = self.r_current
        new_pos = (curr_i + d_i, curr_j + d_j)
        self.r_current = new_pos
        return;

    def copy(self):
        return deepcopy(self);

    def get_board(self):
        return deepcopy(self.grid_world);
