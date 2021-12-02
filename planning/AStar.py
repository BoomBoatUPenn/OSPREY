from numpy.random import random
from copy import deepcopy
from queue import PriorityQueue
from math import sqrt, pi, cos, sin

import ocean

class AStar(object):
    def __init__(self, ocean_map, left, *args, **kwargs):
        self.ocean_map = ocean_map.copy()
        self.left = left
        if left:
            self.start = self.ocean_map.l_start
            self.goal = self.ocean_map.l_goal
        else:
            self.start = self.ocean_map.r_start
            self.goal = self.ocean_map.r_goal
        self.visited = set()
        self.g_scores = {}
        for row in range(self.ocean_map.n):
            for col in range(self.ocean_map.n):
                self.g_scores[(row, col)] = 1.0e10
        return;
    
    def clear(self):
        self.visited = set()
        self.g_scores = {}
        for row in range(self.ocean_map.n):
            for col in range(self.ocean_map.n):
                self.g_scores[(row, col)] = 1.0e10
        return;


    def backtrack(self, start, goal, parents):
        current = goal
        path = []
        path.append(current)
        while current is not start:
            prev = parents[current]
            path.append(prev)
            current = prev
        path.reverse()
        return path;

    def heuristic(self, node, goal, mode):
        if mode == 'manhattan':
            h = abs(node[0] - goal[0])
            h += abs(node[1] - goal[1])
            #h *= 0.1
        elif mode == 'l2':
            h = abs(node[0] - goal[0])**2
            h += abs(node[1] - goal[1])**2
            h = sqrt(h)
        return int(h);

    def distance(self, current, neighbor):
        curr_i, curr_j = current
        neighbor_i, neighbor_j = neighbor
        if abs(neighbor_i - curr_i) == 0 or abs(neighbor_j - curr_j) == 0:
            return 1;
        else:
            return 1;

    def search(self, start, goal, mode):
        frontier = PriorityQueue()
        parents = {}
        f = self.g_scores[start] + self.heuristic(start, goal, mode)
        frontier.put((f, start))
        self.visited.add(start)
        self.g_scores[start] = 0
        parents[start] = None

        while not frontier.empty():
            _, current = frontier.get()
            self.visited.add(current)
            if current == goal:
                path = self.backtrack(start, goal, parents)
                return (path, deepcopy(self.visited));
            for neighbor in self.ocean_map.get_neighbors(self.left, current):
                g = self.g_scores[current] + self.distance(current, neighbor)
                if g < self.g_scores[neighbor]:
                    parents[neighbor] = current
                    self.g_scores[neighbor] = g
                    f = self.g_scores[neighbor] + self.heuristic(neighbor, goal, mode)
                    if neighbor not in self.visited:
                        frontier.put((f, neighbor))
                self.visited.add(neighbor)
        return (None, deepcopy(self.visited));

    def runner(self, tsp=0, mode='manhattan'):
        if tsp:
            waypoints = self.generate_waypoints(tsp)
            path_list, visited_list = self.tsp(self.start, waypoints, mode)
            return (path_list, visited_list);
        else:
            path, visited = self.search(self.start, self.goal, mode)
            return (path, visited);

    def tsp(self, start, waypoints, mode):
        for i, way in enumerate(waypoints):
            if i == 0:
                path, visited = self.search(start, way, mode)
                visited_list = [list(visited)]
                prev = way
            else:
                self.clear()
                next_path, newly_visited = self.search(prev, way, mode)
                path.extend(next_path)
                newly_visited.difference_update(visited)
                visited_list.append(list(newly_visited))
                prev = way
        return (path, visited_list);
    
    def generate_waypoints(self, num_waypoints):
        radius = sqrt((self.ocean_map.l_start[0] - int(self.ocean_map.n / 2))**2 + (self.ocean_map.l_start[1] - int(self.ocean_map.n / 2))**2)
        if self.left:
            radius *= -1.0
        intervals = num_waypoints
        angular_displacement = pi/(intervals+1)
        waypoints = []
        board = self.ocean_map.get_board()
        for i in range(intervals):
            x_new = int(self.ocean_map.n / 2) + int(radius*cos((i+1)*angular_displacement))
            y_new = int(self.ocean_map.n / 2) + int(radius*sin((i+1)*angular_displacement))
            new_waypoint = (x_new, y_new)
            if board[new_waypoint] >= 0:
                waypoints.append(new_waypoint)
        waypoints.reverse()
        waypoints.append(self.goal)
        return waypoints;
