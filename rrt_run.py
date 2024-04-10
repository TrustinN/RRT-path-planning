import timeit
import pyqtgraph as pg

###############################################################################
# Make Test Regions for RRT                                                   #
###############################################################################

from utils.maps.maps2d import RaceMap
from utils.maps.maps2d import SquareObsMap
from utils.maps.maps2d import Maze
from utils.maps.maps3d import RandObsMap

###############################################################################
# Sampling                                                                    #
###############################################################################

from utils.maps.map_utils import Rectangle
from utils.maps.map_utils import Cube

###############################################################################
# RRT Methods and Variations                                                  #
###############################################################################

# from rrt_methods.rrt import rrt_run
# from rrt_methods.rrt_connect import rrt_run
# from rrt_methods.rrt_star import rrt_run
# from rrt_methods.rrt_star_connect import rrt_run
# from rrt_methods.quick_rrt_star import rrt_run
# from rrt_methods.informed_rrt_star import rrt_run
from rrt_methods.informed_quick_rrt_star import rrt_run

###############################################################################
# Generate Regions                                                            #
###############################################################################

dim = 3

# choice = "rm"
choice = "rom"
# choice = "maze"

if dim == 2:
    bounds = [(-200, 1000), (-200, 1000)]
    if choice == "rm":
        map = RaceMap()
    elif choice == "rom":
        map = SquareObsMap(10, 100)
    elif choice == "maze":
        map = Maze(20)
    map.sample_init(Rectangle(bounds))
else:
    bounds = [(-200, 1000), (-200, 1000), (-200, 1000)]
    if choice == "rom":
        map = RandObsMap(25, 180)
    map.sample_init(Cube(bounds))

start = timeit.default_timer()
path = rrt_run(map=map, step_size=20, max_iter=2000, plotting=True)
stop = timeit.default_timer()

print('Time: ', stop - start)
pg.exec()






