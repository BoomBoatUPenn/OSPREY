import sys

import numpy as np
from numpy.random import random
import matplotlib.pyplot as plt
from copy import deepcopy
from queue import PriorityQueue
from math import sqrt, pi, cos, sin

import ocean
import AStar


class PathVisualization(object):

    def __init__(self, left, n, *args, **kwargs):
        self.left = left
        self.n = n
        return;

    def plotter(self):
        ocean_map = ocean.GridWorld(self.n)
        with plt.ion():
            fig, ax = plt.subplots(2, 4, figsize=(20, 15))

            # draw ocean environment
            for i in range(2):
                if i == 0:
                    mode = 'l2'
                else:
                    mode = 'manhattan'
                for j in range(4):
                    if j == 0:
                        # no waypoints
                        board = ocean_map.get_board()
                        ax[i, j].imshow(board)
                        ax[i, j].set_title('A* Search \n (' + mode + ' heuristic)')
                        ax[i, j].axis('off')
                    else:
                        tsp = j
                        # with waypoints
                        multi_board = ocean_map.get_board()
                        ax[i, j].imshow(multi_board)
                        ax[i, j].set_title(str(tsp) + '-Waypoint A* Search \n (' + mode + ' heuristic)')
                        ax[i, j].axis('off')
            plt.show()
            plt.pause(30.0)
            
            paths = {}

            # highlight regions explored by A*
            for i in range(2):
                if i == 0:
                    mode = 'l2'
                else:
                    mode = 'manhattan'
                for j in range(4):
                    ax[i, j].cla()
                    if j == 0:
                        board = ocean_map.get_board()
                        # no waypoints
                        a_star = AStar.AStar(ocean_map, self.left)
                        path, visited = a_star.runner(tsp=False, mode=mode)
                        for v in visited:
                            board[v] = 30
                        ax[i, j].imshow(board)
                        ax[i, j].set_title('A* Search \n (' + mode + ' heuristic)')
                        ax[i, j].axis('off')
                    else:
                        # with waypoints (currently two waypoints including the goal)
                        tsp=j
                        multi_goal = AStar.AStar(ocean_map, self.left)
                        path, multi_visited_list = multi_goal.runner(tsp=tsp, mode=mode)
                        multi_board = ocean_map.get_board()
                        counter = 15
                        for ways in multi_visited_list:
                            for w in ways:
                                multi_board[w] = counter
                            counter *= 2
                        ax[i,j].imshow(multi_board)
                        ax[i, j].set_title(str(tsp) + '-Waypoint A* Search \n (' + mode + ' heuristic)')
                        ax[i, j].axis('off')
                    paths[(i, j)] = path
            plt.draw()
            plt.pause(1.0)

            # animate planned paths
            counter = 0
            while True:
                all_paths = False
                for i in range(2):
                    for j in range(4):
                        if len(paths[(i, j)]) - 1 > counter:
                            all_paths = True
                            path = paths[(i, j)]
                            prev = path[counter]
                            p = path[counter + 1]
                            dy = p[0] - prev[0]
                            dx = p[1] - prev[1]
                            ax[i,j].arrow(prev[1], prev[0], dx, dy)
                
                plt.draw()
                plt.pause(0.1)
                counter += 1
                if not all_paths:
                    break;
        return (fig, ax);

def __main__(n, left):
    #n = 100
    #left = True
    planned_paths = PathVisualization(left, n)
    fig, ax = planned_paths.plotter()

if __name__ == '__main__':
    n = None
    left = None
    opts = [opt for opt in sys.argv[1:] if opt.startswith("-")]
    args = [arg for arg in sys.argv[1:] if not arg.startswith("-")]
    if len(sys.argv) > 1:
        if '-n' in opts:
            n = args[0]
            n = int(n)
        if '-left' in opts:
            left = args[1]
            if left.lower() == 'true':
                left = True
            elif left.lower() == 'false':
                left = False
    if n is None:
        n = 100
    if left is None:
        left = True

    __main__(n, left);