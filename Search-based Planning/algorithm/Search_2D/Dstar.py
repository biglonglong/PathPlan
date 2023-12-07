"""
Dstar - for minest cost_neighbor:
 plan by Dijkstra from goal to source, move by explore_tree with replan crossing path when facing new_obs in dynamic map.
 Defaults cost_neighbor(point, new_obs) is math.inf, process_state will spread the new_obs_info until find another exlore_tree_point.
Attention: facing obs, replan suboptimal path(replan break condition may modify point_path passed for the optimal path, witch make explore_tree error, whose origin is the break condition, so we can set a optimality based on cost_neighbor_interval)
"""

import math
import heapq
import numpy as np
import matplotlib.pyplot as plt

import os
import sys
import time
import threading
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + r"\..\..")
from map import Plotting
from map import Env_Base

class dstar:
    def __init__(self, source, goal, optimality):
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

        self.optimality = optimality

    def get_neighbor(self, point):
        return [(point[0] + move[0], point[1] + move[1]) for move in self.motions]

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

                # close_neighbor finded & cost_neighbor(neighbor,ep) is OK, change ep path to close_neighbor
                if mink_ep < self.h[ep]:
                    if self.h[neighbor] <= mink_ep and self.h[ep] > self.h[neighbor] + self.cost_neighbor(neighbor, ep):
                        self.explore_tree[ep] = neighbor
                        self.h[ep] = self.h[neighbor] + self.cost_neighbor(neighbor, ep)

                # condition2:dijkstra optima 
                # condition3:cost_neighbor(stuck_point,obs) spread to h[stuck_point]
                if mink_ep == self.h[ep]:
                    if (self.t[neighbor] == 'NEW') or \
                        (self.explore_tree[neighbor] != ep and self.h[neighbor] > self.h[ep] + self.cost_neighbor(neighbor, ep)) or \
                        (self.explore_tree[neighbor] == ep and self.h[neighbor] != self.h[ep] + self.cost_neighbor(neighbor, ep)):
                        self.explore_tree[neighbor] = ep
                        self.insert(neighbor, self.h[ep] + self.cost_neighbor(neighbor, ep))
                
                else:
                    # condition2:cost_neighbor(stuck_point,stuck_point2) spread to h[stuck_point2]
                    if (self.t[neighbor] == 'NEW') or \
                        (self.explore_tree[neighbor] == ep and self.h[neighbor] != self.h[ep] + self.cost_neighbor(neighbor, ep)):
                        self.explore_tree[neighbor] = ep
                        self.insert(neighbor, self.h[ep] + self.cost_neighbor(neighbor, ep))
                    
                    # ep finded & cost_neighbor(neighbor,ep) is OK, change neighbor path to ep(dijkstra optima)
                    elif self.explore_tree[neighbor] != ep and self.h[neighbor] > self.h[ep] + self.cost_neighbor(neighbor, ep):
                        self.insert(ep, self.h[ep])
                    
                    # far_neighbor finded & cost_neighbor(neighbor,ep) is OK, change ep path to far_neighbor(dijkstra optima)
                    elif self.explore_tree[neighbor] != ep and self.h[ep] > self.h[neighbor] + self.cost_neighbor(neighbor, ep) and self.t[neighbor] == 'CLOSED' and self.h[neighbor] > mink_ep:
                        self.insert(neighbor, self.h[neighbor])
                    
                    else:
                        pass

        return self.open_set[0][0]

    def plan(self):
        self.t[self.goal] = 'NEW'
        self.insert(self.goal, 0)
        self.explore_tree[self.goal] = self.goal

        while True:
            self.process_state()
            if self.t.get(self.source) == 'CLOSED':
                break

        return self.extract_path(), self.closed_set

    def replan(self, point_path):
        if self.t[point_path] == 'CLOSED':
            self.insert(self.explore_tree[point_path], self.h[self.explore_tree[point_path]]) 
        
        while True:
            mink_ep = self.process_state()
            if mink_ep >= self.h[point_path] + self.optimality:
                break

    def on_press(self, event, plot):
        x, y = round(event.xdata), round(event.ydata)
        if x < 0 or x > self.env.x_range - 1 or y < 0 or y > self.env.y_range - 1:
            print("error area!")
        else:
            if (x, y) not in self.obs:
                self.obs.add((x, y))
                plot.update_obs_dynamic((x, y))
                print("add obstacle at: ", (x, y))

                start_time = time.time()
                path = [self.source]

                point_path = self.source
                while point_path !=self.goal:
                    if time.time() - start_time > 5.0:
                        print("replan timeout! (may be difficult to find a path)")
                        plt.close('all')
                        os._exit(0)

                    if self.is_collision(point_path, self.explore_tree[point_path]):
                        thread = threading.Thread(target=self.replan, args=(point_path,))
                        thread.start()
                    else:
                        point_path = self.explore_tree[point_path]
                        path.append(point_path)

                plot.animation("D*", path, False, "Dstar", [])
                plt.gcf().canvas.draw_idle()
                
def main():
    source = (5, 5)
    goal = (45, 25)

    # speed replan break condition, such as optimality = -1.5
    optimality = 0

    DStar = dstar(source, goal, optimality)
    plot = Plotting.plotting(source, goal)
    path, visited = DStar.plan()
    plot.animation("D*", path, False, "Dstar", visited)

    plt.gcf().canvas.mpl_connect('button_press_event',  lambda event: DStar.on_press(event, plot=plot))
    plt.show()

if __name__ == '__main__':
    main()