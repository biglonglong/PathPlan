"""
Dstar - for minest cost_neighbor:
 plan by Dijkstra from goal to source, move by explore_tree with replan crossing path when facing new_obs in dynamic map.
 Defaults cost_neighbor(point, new_obs) is math.inf, process_state will spread the new_obs_info until find another exlore_tree_point.
Attention: k as the comparsion of h for whether h is modified
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

class dstar:
    def __init__(self, source, goal):
        self.env = Env_Base.env()
        self.obs = self.env.obs
        self.source = source
        self.goal = goal

        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.open_set = []
        self.closed_set = []
        self.t = dict()         # state
        self.h = dict()         # current_cost_goal
        self.k = dict()         # minest_cost_goal

        self.explore_tree = dict()
        self.path = []

    def get_neighbor(self, point):
        return [(point[0] + move[0], point[1] + move[1]) for move in self.motions]

    def is_collision(self, start, end):
        if start in self.obs or end in self.obs:
            return True
        
        current_x,current_y,end_x,end_y = start[0],start[1],end[0],end[1]
        x_change = (end_x - current_x) / max(abs(end_x - current_x),1)
        y_change = (end_y - current_y) / max(abs(end_y - current_y),1)       

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
        path = [self.source]
        point_path = self.source

        while True:
            point_path = self.explore_tree[point_path]
            path.append(point_path)
            
            if point_path == self.goal:
                break
        
        return list(path)

    def insert(self, point, new_h):
        if self.t[point] == 'NEW':
            self.k[point] = new_h
        elif self.t[point] == 'OPEN':
            self.k[point] = min(self.k[point], new_h)
        else:
            self.k[point] = min(self.h[point], new_h)   # update k as h after obs-path-update

        self.h[point] = new_h

        self.t[point] = 'OPEN'

        heapq.heappush(self.open_set,
                       (self.k[point], point))

    def process_state(self):
        if self.open_set:
            mink_ep, ep = heapq.heappop(self.open_set)
            self.closed_set.append(ep)
            self.t[ep] = 'CLOSED'

            for neighbor in self.get_neighbor(ep):
                if neighbor not in self.h:
                    self.t[neighbor] = 'NEW'
                    self.h[neighbor] = math.inf
                    self.k[neighbor] = math.inf

                # condition1&2:dijkstra optima 
                if mink_ep == self.h[ep]:
                    if (self.t[neighbor] == 'NEW') or \
                        (self.explore_tree[neighbor] != ep and self.h[neighbor] > self.h[ep] + self.cost_neighbor(neighbor, ep)):
                        self.explore_tree[neighbor] = ep
                        self.insert(neighbor, self.h[ep] + self.cost_neighbor(neighbor, ep))
     
    def plan(self):
        self.t[self.goal] = 'NEW'
        self.insert(self.goal, 0)
        self.explore_tree[self.goal] = self.goal

        while True:
            self.process_state()
            if self.t.get(self.source) == 'CLOSED':
                break
    
        self.path = self.extract_path()

        return self.path, self.closed_set

    def on_press(self, event, plot):
        x, y = round(event.xdata), round(event.ydata)
        if x < 0 or x > self.env.x_range - 1 or y < 0 or y > self.env.y_range - 1:
            print("error area!")
        else:
            if (x, y) not in self.obs:
                self.obs.add((x, y))
                plot.update_obs(self.obs)
                plot.animation("D*", self.path, "Dstar", self.closed_set)
                
                plt.gcf().canvas.draw_idle()
                print("add obstacle at: ", (x, y))
                

def main():
    source = (5, 5)
    goal = (45, 25)

    DStar = dstar(source, goal)
    plot = Plotting.plotting(source, goal)
    path, visited = DStar.plan()
    plot.animation("D*", path, "Dstar", visited)

    plt.gcf().canvas.mpl_connect('button_press_event',  lambda event: DStar.on_press(event, plot=plot))
    plt.show()

if __name__ == '__main__':
    main()

    # def on_press(self, event):
    #     x, y = event.xdata, event.ydata
    #     if x < 0 or x > self.x - 1 or y < 0 or y > self.y - 1:
    #         print("Please choose right area!")
    #     else:
    #         x, y = int(x), int(y)
    #         if (x, y) not in self.obs:
    #             print("Add obstacle at: s =", x, ",", "y =", y)
    #             self.obs.add((x, y))
    #             self.Plot.update_obs(self.obs)

    #             s = self.s_start
    #             self.visited = set()
    #             self.count += 1

    #             while s != self.s_goal:
    #                 if self.is_collision(s, self.PARENT[s]):
    #                     self.modify(s)
    #                     continue
    #                 s = self.PARENT[s]

    #             self.path = self.extract_path(self.s_start, self.s_goal)

    #             plt.cla()
    #             self.Plot.plot_grid("Dynamic A* (D*)")
    #             self.plot_visited(self.visited)
    #             self.plot_path(self.path)

    #         self.fig.canvas.draw_idle()

    # def modify(self, s):
    #     """
    #     start processing from state s.
    #     :param s: is a node whose status is RAISE or LOWER.
    #     """

    #     self.modify_cost(s)

    #     while True:
    #         k_min = self.process_state()

    #         if k_min >= self.h[s]:
    #             break 

    # def modify_cost(self, s):
    #     # if node in CLOSED set, put it into OPEN set.
    #     # Since cost may be changed between s - s.parent, calc cost(s, s.p) again

    #     if self.t[s] == 'CLOSED':
    #         self.insert(s, self.h[self.PARENT[s]] + self.cost(s, self.PARENT[s]))


    # def plot_path(self, path):
    #     px = [x[0] for x in path]
    #     py = [x[1] for x in path]
    #     plt.plot(px, py, linewidth=2)
    #     plt.plot(self.s_start[0], self.s_start[1], "bs")
    #     plt.plot(self.s_goal[0], self.s_goal[1], "gs")

    # def plot_visited(self, visited):
    #     color = ['gainsboro', 'lightgray', 'silver', 'darkgray',
    #              'bisque', 'navajowhite', 'moccasin', 'wheat',
    #              'powderblue', 'skyblue', 'lightskyblue', 'cornflowerblue']

    #     if self.count >= len(color) - 1:
    #         self.count = 0

    #     for x in visited:
    #         plt.plot(x[0], x[1], marker='s', color=color[self.count])
