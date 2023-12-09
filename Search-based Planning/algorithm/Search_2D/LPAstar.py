"""
LPAstar - :
 
 Defaults
Attention: latter search 
"""

import math
import heapq
import numpy as np
import matplotlib.pyplot as plt

import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + r"\..\..")
from map import Plotting
from map import Env_Base

class lpastar:
    def __init__(self, source, goal):
        self.env = Env_Base.env()
        self.obs = self.env.obs
        self.source = source
        self.goal = goal

        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.open_set = []
        self.closed_set = []
        self.g = dict()      # suitable explore_base after compute_shortest_path
        self.rhs = dict()    # real-time cost_source

    def cost_heuristic(self, point, heuristic_type = "euclidean"):
        if point in self.obs:
            return math.inf

        goal = self.goal

        if heuristic_type == "euclidean":
            return math.hypot(goal[0] - point[0], goal[1] - point[1])
        else:
            return abs(goal[0] - point[0]) + abs(goal[1] - point[1])

    def get_neighbor(self, point):
        neighbors = []
        for move in self.motions:
            neighbor = point[0] + move[0], point[1] + move[1]
            if neighbor[0] >=0 and neighbor[0] < self.env.x_range and neighbor[1] >=0 and neighbor[1] < self.env.y_range:
                neighbors.append(neighbor)
        
        return neighbors

    def is_collision(self, start, end):
        if start in self.obs or end in self.obs:
            return True
        
        current_x,current_y,end_x,end_y = start[0],start[1],end[0],end[1]
        x_change = (end_x - current_x) / max(abs(end_x - current_x),1)
        y_change = (end_y - current_y) / max(abs(end_y - current_y),1)  

        while(current_x != end_x and current_y != end_y):
            current_x += x_change
            current_y += y_change
            if (current_x,current_y) in self.obs:
                return True

        while(current_x != end_x):
            current_x += x_change
            if (current_x,current_y) in self.obs:
                return True
        while(current_y != end_y):
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
            min_cost = math.inf
            closest_neighbor = None
            for neighbor in self.get_neighbor(point_path):
                cost_point_path = self.g[neighbor] + self.cost_neighbor(neighbor, point_path)
                if cost_point_path < min_cost:
                    min_cost = cost_point_path
                    closest_neighbor = neighbor

            point_path = closest_neighbor
            path.append(point_path)

            if point_path == self.source:
                break
        
        return list(path)

    def calculate_key(self, point):
        return min(self.g[point], self.rhs[point]) + self.cost_heuristic(point), min(self.g[point], self.rhs[point])

    def torrent(self):
        for i in range(self.env.x_range):
            for j in range(self.env.y_range):
               self.g[(i, j)] = math.inf
               self.rhs[(i, j)] = math.inf

    # update new_rhs of point and push into open_set 
    # condition1: update-path -> consistent
    # condition2: unupdate-path -> consistent
    # condition3: update-path -> inconsistent -> repush
    def update_rhs(self, point):   
        if point != self.source:
            self.rhs[point] = min(self.g[neighbor] + self.cost_neighbor(neighbor, point)
                                    for neighbor in self.get_neighbor(point))
        
        for item in self.open_set:
            if item[1] == point:
                self.open_set.remove(item)
                break
        if self.g[point] != self.rhs[point]:
            heapq.heappush(self.open_set, (self.calculate_key(point), point))
 
    # update g suitable explore_base for extract(change path blocked)
    def compute_shortest_path(self):
        while True:
            min_key, explore_point = heapq.heappop(self.open_set)
            self.closed_set.append(explore_point)

            # cost_goal with no-obs-env < cost_goal with obs-env
            if min_key >= self.calculate_key(self.goal) and self.g[self.goal] == self.rhs[self.goal]:
                break

            # optimaed&update2consistent
            if self.g[explore_point] > self.rhs[explore_point]:  
                self.g[explore_point] = self.rhs[explore_point]
            # update2unconsistent&low_rank_openset&waiting to optima
            else:
                self.g[explore_point] = math.inf       
                self.update_rhs(explore_point)   

            for neighbor in self.get_neighbor(explore_point):
                self.update_rhs(neighbor)

    def plan(self):
        self.torrent()
        self.rhs[self.source] = 0
        heapq.heappush(self.open_set, (self.calculate_key(self.source), self.source))

        self.compute_shortest_path()

        return self.extract_path(), self.closed_set

    def replan(self, changed_point):
        self.update_rhs(changed_point)

        for neighbor in self.get_neighbor(changed_point):
            self.update_rhs(neighbor)

        self.compute_shortest_path()

    def on_press(self, event, plot):
        x, y = round(event.xdata), round(event.ydata)
        if x < 0 or x > self.env.x_range - 1 or y < 0 or y > self.env.y_range - 1:
            print("error area!")
        else:
            if (x, y) not in self.obs:
                self.obs.add((x, y))
                print("add obstacle at: ", (x, y))
            else:
                self.obs.remove((x, y))
                print("remove obstacle at: ", (x, y))
            plot.update_obs_dynamic((x, y))

            self.replan((x, y))

            plot.animation("Lifelong_Planning_Astar (LPA*)", self.extract_path(), False, "LPAstar", [])
            plt.gcf().canvas.draw_idle()
                
def main():
    source = (5, 5)
    goal = (45, 25)

    LPatar = lpastar(source, goal)
    plot = Plotting.plotting(source, goal)
    path, visited = LPatar.plan()
    plot.animation("Lifelong_Planning_Astar (LPA*)", path, False, "LPAstar", visited)

    plt.gcf().canvas.mpl_connect('button_press_event',  lambda event: LPatar.on_press(event, plot=plot))
    plt.show()

if __name__ == '__main__':
    main()