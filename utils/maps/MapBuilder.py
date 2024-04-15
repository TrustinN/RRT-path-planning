import numpy as np


class MapBuilder():
    def __init__(self, dim, seed=None, map="rom"):
        self.name = map
        self.seed = seed
        if seed:
            np.random.seed(seed)

        self.dim = dim
        if self.dim == 2:
            import utils.maps.maps2d as maps

        elif self.dim == 3:
            import utils.maps.maps3d as maps

        self.maps = maps

        self.new_map()
        self.map.reset()

    def new_map(self):
        if self.name == "rom":
            self.map = self.maps.RandomObsMap(10, 150)

    def get_map(self):
        return self.map







