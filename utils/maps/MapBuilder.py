import utils.maps.maps2d as maps2d
import utils.maps.maps3d as maps3d
import numpy as np


class MapBuilder():

    def redefine_params(self, dim, seed, name, args):
        self.dim = dim
        if dim == 2:
            self.maps = maps2d

        elif dim == 3:
            self.maps = maps3d

        self.name = name
        self.seed = seed
        if seed:
            np.random.seed(seed)

        self.args = args
        self.new_map()
        self.map.reset()

    def new_map(self):
        if self.name == "rom":
            self.map = self.maps.RandomObsMap(self.args[0], self.args[1])

    def get_map(self):
        return self.map







