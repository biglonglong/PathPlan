'''
Astar - for minest cost_total:
 cost_heuristic&explore_base to guide, combine explore_base(inconsistent) from cost_neighbor of get_neighbor.
 Defaults unexplored explore_base to None, exploring to math.inf(obs), explore in minest-cost-order.
Attention: maintain consistency(cost_heuristic is usually designed not to overestimate actual costs)
'''

import math
import heapq
import numpy as np

import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + r"\..\..")
from map import Plotting
from map import Env

class astar:
    def __init__(self, source, goal):
        self.env = Env.env()
        self.obs = self.env.obs
        self.source = source
        self.goal = goal

        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.open_set = []
        self.close_set = []
        self.explore_base = dict()
        self.explore_tree = dict()

    def cost_heuristic(self, point, heuristic_type = "euclidean"):
        if point in self.obs:
            return math.inf

        goal = self.goal

        if heuristic_type == "euclidean":
            return math.hypot(goal[0] - point[0], goal[1] - point[1])
        else:
            return abs(goal[0] - point[0]) + abs(goal[1] - point[1])

    def cost_total(self, point):
        return self.explore_base[point] + self.cost_heuristic(point)

    def get_neighbor(self, point):
        return [(point[0] + move[0], point[1] + move[1]) for move in self.motions]

    def is_collision(self, start, end):
        if start in self.obs or end in self.obs:
            return True
        
        current_x,current_y,end_x,end_y = start[0],start[1],end[0],end[1]
        x_change = end_x - current_x / max(abs(end_x - current_x),1)
        y_change = end_y - current_y / max(abs(end_y - current_y),1)       

        while(current_x != end_x or current_y != end_y):
            current_x += x_change
            current_y += y_change
            if (current_x,current_y) in self.obs:
                return True
        return False
        
    def cost_neighbor(self, start, end, neighbor_type = "diagonal"):
        if self.is_collision(start, end):
            return math.inf
        
        x_dis = abs(end[0] - start[0])
        y_dis = abs(end[1] - start[1])

        if neighbor_type == "diagonal":
            return x_dis + y_dis + (np.sqrt(2) - 2) * min(x_dis, y_dis)
        else:
            return x_dis + y_dis

    def extract_path(self):
        path = [self.goal]
        point_path = self.goal

        while True:
            point_path = self.explore_tree[point_path]
            path.append(point_path)
            
            if point_path == self.source:
                break
        
        return list(path)

    def searching(self):
        self.explore_base[self.source] = 0
        self.explore_tree[self.source] = self.source
        
        heapq.heappush(self.open_set,
                       (self.cost_total(self.source), self.source))

        while self.open_set:
            _, explore_point = heapq.heappop(self.open_set)
            self.close_set.append(explore_point)
            
            if explore_point == self.goal:  
                break

            for neighbor in self.get_neighbor(explore_point):
                new_cost = self.explore_base[explore_point] + self.cost_neighbor(explore_point, neighbor)

                if neighbor not in self.explore_base:
                    self.explore_base[neighbor] = math.inf
                if new_cost < self.explore_base[neighbor]:
                    self.explore_base[neighbor] = new_cost
                    self.explore_tree[neighbor] = explore_point
                    heapq.heappush(self.open_set, (self.cost_total(neighbor), neighbor))

        return self.extract_path(), self.close_set

def main():
    source = (5, 5)
    goal = (45, 25)

    AStar = astar(source, goal)
    plot = Plotting.plotting(source, goal)
    path, visited = AStar.searching()
    plot.animation("A*", path, "Astar", visited)

if __name__ == '__main__':
    main()