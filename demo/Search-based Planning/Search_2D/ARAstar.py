"""

@description: local inconsistency: g-value decreased.
g(s) decreased introduces a local inconsistency between s and its successors.

"""


class AraStar:
    def __init__(self, s_start, s_goal, e, heuristic_type):
        self.e = e                            # weight

        self.INCONS = {}                      # INCONSISTENT set
        self.path = []                        # planning path
        self.visited = []                     # order of visited nodes

    def init(self):
        """
        initialize each set.
        """

        self.g[self.s_start] = 0.0

        self.g[self.s_goal] = math.inf
        self.OPEN[self.s_start] = self.f_value(self.s_start)

        self.PARENT[self.s_start] = self.s_start

    def searching(self):
        self.init()
        self.ImprovePath()
        self.path.append(self.extract_path())

        while self.update_e() > 1:                                          # continue condition
            self.e -= 0.4                                                   # increase weight
            self.OPEN.update(self.INCONS)
            self.OPEN = {s: self.f_value(s) for s in self.OPEN}             # update f_value of OPEN set

            self.INCONS = dict()
            self.CLOSED = set()
            self.ImprovePath()                                              # improve path
            self.path.append(self.extract_path())

        return self.path, self.visited

    def ImprovePath(self):
        """
        :return: a e'-suboptimal path
        """

        visited_each = []

        while True:
            s, f_small = self.calc_smallest_f()

            if self.f_value(self.s_goal) <= f_small:
                break

            self.OPEN.pop(s)
            self.CLOSED.add(s)

            for s_n in self.get_neighbor(s):
                if s_n in self.obs:
                    continue

                new_cost = self.g[s] + self.cost(s, s_n)

                if s_n not in self.g or new_cost < self.g[s_n]:
                    self.g[s_n] = new_cost
                    self.PARENT[s_n] = s
                    visited_each.append(s_n)

                    if s_n not in self.CLOSED:
                        self.OPEN[s_n] = self.f_value(s_n)
                    else:
                        self.INCONS[s_n] = 0.0

        self.visited.append(visited_each)

    def calc_smallest_f(self):
        """
        :return: node with smallest f_value in OPEN set.
        """

        s_small = min(self.OPEN, key=self.OPEN.get)

        return s_small, self.OPEN[s_small]

    def update_e(self):
        v = float("inf")

        if self.OPEN:
            v = min(self.g[s] + self.h(s) for s in self.OPEN)
        if self.INCONS:
            v = min(v, min(self.g[s] + self.h(s) for s in self.INCONS))

        return min(self.e, self.g[self.s_goal] / v)


def main():
    s_start = (5, 5)
    s_goal = (45, 25)

    arastar = AraStar(s_start, s_goal, 2.5, "euclidean")
    plot = plotting.Plotting(s_start, s_goal)

    path, visited = arastar.searching()
    plot.animation_ara_star(path, visited, "Anytime Repairing A* (ARA*)")


if __name__ == '__main__':
    main()
