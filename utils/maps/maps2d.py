import numpy as np
from .map_utils import Map
from .map_utils import SampleScope
from utils.rtree.rtree_utils import Rect
from utils.quickhull.hull import QuickHull
import pyqtgraph as pg


class Map2d(Map):
    def __init__(self, obstacles, dim):
        super().__init__(obstacles, dim)

    def intersects_line(self, line):

        start = line[0]
        end = line[1]
        l_bound = Rect.combine_points([start, end])
        potential = self.obs_tree.SearchOverlap(l_bound)

        for f in potential:
            if f.intersects_line(start, end):
                return True

        return False

    def intersections(self, line):
        ints = []
        l_bound = Rect.combine_points(line)
        potential = self.obs_tree.SearchOverlap(l_bound)

        for o in potential:
            inter = o.intersections(line)

            if type(inter) is np.ndarray:
                ints.append(inter)

        return ints

    def plot(self, view):
        for o in self.obstacles:
            o.plot(view)

    def plot_path(self, path, view):
        for i in range(len(path) - 1):
            line = pg.PlotDataItem(np.array([path[i], path[i + 1]]),
                                   connect="pairs",
                                   pen=pg.mkPen("#ff00ff"),)
            view.addItem(line)


class RandomObsMap(Map2d):
    def __init__(self, n, size):

        start_pos = np.array([10, 10])
        end_pos = np.array([790, 790])

        ospace = SampleScope.Rectangle([400, 400], 500, 500)
        obstacles = []

        while n > 0:
            center = ospace.sample()
            r = SampleScope.Rectangle(center, size, size)

            o = QuickHull([r.sample() for i in range(10)])

            if not o.contains_point(start_pos) and not o.contains_point(end_pos):
                obstacles.append(o)
                n -= 1

        super().__init__(obstacles=obstacles, dim=2)
        self.add_path([start_pos, end_pos])

    def reset(self):
        self.sample_init(SampleScope.Rectangle([400, 400], 600, 600))


class Maze(Map2d):

    def __init__(self, division):
        # Will turn a 1000 x 1000 grid into one of 1000/div x 1000/div
        grid_length = 1000 / division
        self.grid_length = grid_length
        self.start_pos = np.array([grid_length / 2 for i in range(2)])
        self.end_pos = np.array([1000, 1000]) - np.array([grid_length / 2,
                                                          grid_length / 2,
                                                          ])

        # initialize division x division sized array of not visited
        self.division = division
        self.init_graph()
        self.dfs(len(self.visited) - 1, 0)
        self.make_obstacles()

        super().__init__(obstacles=self.obstacles, dim=2)
        self.add_path([self.start_pos, self.end_pos])

    def init_graph(self):
        self.visited = np.zeros((self.division, self.division))
        self.h_walls = np.ones((self.division - 1, self.division))
        self.v_walls = np.ones((self.division, self.division - 1))

    def dfs(self, y, x):

        self.visited[y][x] = 1
        bag = [0, 1, 2, 3]
        np.random.shuffle(bag)

        while bag:
            choice = bag.pop()

            # Move right one square
            if choice == 0:
                if 0 <= x + 1 < self.division:
                    if not self.visited[y, x + 1]:
                        self.v_walls[y, x] = 0
                        self.visited[y, x + 1] = 1
                        self.dfs(y, x + 1)

            # Move left one square
            if choice == 1:
                if 0 <= x - 1 < self.division:
                    if not self.visited[y, x - 1]:
                        self.v_walls[y, x - 1] = 0
                        self.visited[y, x - 1] = 1
                        self.dfs(y, x - 1)

            # Move up one square
            if choice == 2:
                if 0 <= y + 1 < self.division:
                    if not self.visited[y + 1, x]:
                        self.h_walls[y, x] = 0
                        self.visited[y + 1, x] = 1
                        self.dfs(y + 1, x)

            # Move down one square
            if choice == 3:
                if 0 <= y - 1 < self.division:
                    if not self.visited[y - 1, x]:
                        self.h_walls[y - 1, x] = 0
                        self.visited[y - 1, x] = 1
                        self.dfs(y - 1, x)

    def make_obstacles(self):

        length = self.grid_length / 6
        self.obstacles = []

        for i in range(self.division):
            for j in range(self.division - 1):
                if self.v_walls[len(self.v_walls) - i - 1][j]:
                    # make wall here
                    min_x = (j + 1) * self.grid_length - length / 2
                    max_x = (j + 1) * self.grid_length + length / 2

                    min_y = i * self.grid_length
                    max_y = (i + 1) * self.grid_length

                    wall = QuickHull([np.array([min_x, min_y]),
                                      np.array([max_x, min_y]),
                                      np.array([min_x, max_y]),
                                      np.array([max_x, max_y]),
                                      ])

                    self.obstacles.append(wall)

        for i in range(self.division - 1):
            for j in range(self.division):
                if self.h_walls[len(self.h_walls) - i - 1][j]:
                    # make wall here

                    min_x = j * self.grid_length
                    max_x = (j + 1) * self.grid_length

                    min_y = (i + 1) * self.grid_length - length / 2
                    max_y = (i + 1) * self.grid_length + length / 2

                    wall = QuickHull([np.array([min_x, min_y]),
                                      np.array([max_x, min_y]),
                                      np.array([min_x, max_y]),
                                      np.array([max_x, max_y]),
                                      ])

                    self.obstacles.append(wall)

        # make perimeter
        for i in range(self.division):
            min_x = i * self.grid_length
            max_x = (i + 1) * self.grid_length

            min_y = -length / 2
            max_y = length / 2

            wall = QuickHull([np.array([min_x, min_y]),
                              np.array([max_x, min_y]),
                              np.array([min_x, max_y]),
                              np.array([max_x, max_y]),
                              ])

            self.obstacles.append(wall)

            min_x = i * self.grid_length
            max_x = (i + 1) * self.grid_length

            min_y = self.division * self.grid_length - length / 2
            max_y = self.division * self.grid_length + length / 2

            wall = QuickHull([np.array([min_x, min_y]),
                              np.array([max_x, min_y]),
                              np.array([min_x, max_y]),
                              np.array([max_x, max_y]),
                              ])

            self.obstacles.append(wall)

            min_x = self.division * self.grid_length - length / 2
            max_x = self.division * self.grid_length + length / 2

            min_y = i * self.grid_length
            max_y = (i + 1) * self.grid_length

            wall = QuickHull([np.array([min_x, min_y]),
                              np.array([max_x, min_y]),
                              np.array([min_x, max_y]),
                              np.array([max_x, max_y]),
                              ])

            self.obstacles.append(wall)

            min_x = -length / 2
            max_x = length / 2

            min_y = i * self.grid_length
            max_y = (i + 1) * self.grid_length

            wall = QuickHull([np.array([min_x, min_y]),
                              np.array([max_x, min_y]),
                              np.array([min_x, max_y]),
                              np.array([max_x, max_y]),
                              ])

            self.obstacles.append(wall)

    def reset(self):
        self.sample_init(SampleScope.Rectangle([500, 500], 500, 500))













