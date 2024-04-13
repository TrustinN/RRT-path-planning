import pyqtgraph as pg
from rrt_methods.RRTsolver import RRTsolver

###############################################################################
# Make Test Regions for RRT                                                   #
###############################################################################

from utils.maps.maps2d import RaceMap
from utils.maps.maps2d import RandomObsMap
from utils.maps.maps2d import Maze
from utils.maps.maps3d import RandObsMap

###############################################################################
# Sampling                                                                    #
###############################################################################

from utils.maps.map_utils import Rectangle
from utils.maps.map_utils import Cube

###############################################################################
# Generate Regions                                                            #
###############################################################################

dim = 2

# choice = "rm"
choice = "rom"
# choice = "maze"

if dim == 2:
    bounds = [(-200, 1000), (-200, 1000)]
    if choice == "rm":
        map = RaceMap()
    elif choice == "rom":
        map = RandomObsMap(10, 100)
    elif choice == "maze":
        map = Maze(20)
    map.sample_init(Rectangle(bounds))
else:
    bounds = [(-200, 1000), (-200, 1000), (-200, 1000)]
    if choice == "rom":
        map = RandObsMap(25, 180)
    map.sample_init(Cube(bounds))

methods = ["rrt",                        # 0
           "rrt_connect",                # 1
           "rrt_star",                   # 2
           "rrt_star_connect",           # 3
           "informed_rrt_star",          # 4
           "quick_rrt_star",             # 5
           "informed_quick_rrt_star"     # 6
           ]

rrt = RRTsolver(map=map, step_size=20, max_iter=1000, method=methods[3])
rrt.plot(branches=False, leaves=False)

print(rrt.get_time())

pg.exec()







