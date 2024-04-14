import numpy as np
from utils.maps.map_utils import SampleScope


class MapBuilder():
    def __init__(self, dim, seed=None, map="rom"):
        self.name = map
        self.seed = seed
        if seed:
            np.random.seed(seed)

        if dim == 2:
            import utils.maps.maps2d as maps

        elif dim == 3:
            import utils.maps.maps3d as maps

        if map == "rom":
            self.map = maps.RandomObsMap(10, 180)

        if dim == 2:
            self.map.sample_init(SampleScope.Rectangle([400, 400], 600, 600))

        elif dim == 3:
            self.map.sample_init(SampleScope.Cube([400, 400, 400], 600, 600, 600))

    def get_map(self):
        return self.map







