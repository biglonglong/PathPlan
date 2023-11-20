import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from map import Env

class plotting:
    def __init__(self, source, goal):
        self.source, self.goal = source, goal
        self.env = Env.env()
        self.obs = self.env.obs
        self.ims = []

    def plot_env(self, name):
        obs_x = [obs[0] for obs in self.obs]
        obs_y = [obs[1] for obs in self.obs]

        plt.title(name)
        plt.axis("equal")
        plt.plot(self.source[0], self.source[1], color="blue", marker="s")
        plt.plot(self.goal[0], self.goal[1], color="green", marker="s")
        plt.plot(obs_x, obs_y, "ks")

    def plot_visited(self, *args):
        count = 0
        length = 40
        plot_explore_points = []

        if len(args) == 1:
            if self.source in args[0]:
                args[0].remove(self.source)
            if self.goal in args[0]:
                args[0].remove(self.goal)

            for point in args[0]:
                count += 1
                plot_explore_point = plt.plot(point[0], point[1], color="gray", marker='s')
                plot_explore_points = plot_explore_points + plot_explore_point

                if count % length == 0 or count == len(args[0]):
                    self.ims.append(plot_explore_points)
                    plt.pause(0.01)
        else:
            if self.source in args[0]:
                args[0].remove(self.source)
            if self.goal in args[1]:
                args[1].remove(self.goal)

            len_visited_for, len_visited_back = len(args[0]), len(args[1])
            for i in range(max(len_visited_for, len_visited_back)):
                if i < len_visited_for:
                    count += 1
                    plot_explore_point_for = plt.plot(args[0][i][0], args[0][i][1], color="gray", marker='s')
                    plot_explore_points = plot_explore_points + plot_explore_point_for
                if i < len_visited_back:
                    count += 1
                    plot_explore_point_back = plt.plot(args[1][i][0], args[1][i][1], color="gray", marker='s')
                    plot_explore_points = plot_explore_points + plot_explore_point_back

                if count % length == 0 or count == len(args[0]+args[1]):
                    self.ims.append(plot_explore_points)
                    plt.pause(0.01)

    def plot_path(self, path):
        path_x = [path[i][0] for i in range(len(path))]
        path_y = [path[i][1] for i in range(len(path))]
        
        plot_path = plt.plot(path_x, path_y, color='r', linewidth='3')
        self.ims.append(self.ims[-1] + plot_path)
        plt.pause(1.0)

    def animation(self, name, path, gifname="test", *args):
        fig = plt.figure()

        plt.gcf().canvas.mpl_connect('key_release_event',
                        lambda event: [exit(0) if event.key == 'escape' else None])
        
        self.plot_env(name)

        if len(args) == 1:
            self.plot_visited(args[0])
        else:
            self.plot_visited(args[0], args[1])
        
        self.plot_path(path)
        
        # ani = animation.ArtistAnimation(fig, self.ims, interval=100,
        #                                     repeat_delay=1000, blit=True)
        # ani.save(os.path.dirname(os.path.abspath(__file__)) + rf"\gif\{gifname}.gif",
        #             writer="pillow")
        
        plt.show()



    # @staticmethod
    # def color_list():
    #     cl_v = ['silver',
    #             'wheat',
    #             'lightskyblue',
    #             'royalblue',
    #             'slategray']
    #     cl_p = ['gray',
    #             'orange',
    #             'deepskyblue',
    #             'red',
    #             'm']
    #     return cl_v, cl_p

    # @staticmethod
    # def color_list_2():
    #     cl = ['silver',
    #           'steelblue',
    #           'dimgray',
    #           'cornflowerblue',
    #           'dodgerblue',
    #           'royalblue',
    #           'plum',
    #           'mediumslateblue',
    #           'mediumpurple',
    #           'blueviolet',
    #           ]
    #     return cl

    # def animation_lrta(self, path, visited, name):
    #     self.plot_grid(name)
    #     cl = self.color_list_2()
    #     path_combine = []

    #     for k in range(len(path)):
    #         self.plot_visited(visited[k], cl[k])
    #         plt.pause(0.2)
    #         self.plot_path(path[k])
    #         path_combine += path[k]
    #         plt.pause(0.2)
    #     if self.source in path_combine:
    #         path_combine.remove(self.source)
    #     self.plot_path(path_combine)
    #     plt.show()

    # def animation_ara_star(self, path, visited, name):
    #     self.plot_grid(name)
    #     cl_v, cl_p = self.color_list()

    #     for k in range(len(path)):
    #         self.plot_visited(visited[k], cl_v[k])
    #         self.plot_path(path[k], cl_p[k], True)
    #         plt.pause(0.5)

    #     plt.show()



