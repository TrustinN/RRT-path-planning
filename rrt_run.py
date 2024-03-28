import timeit

###############################################################################
# Make Test Regions for RRT                                                   #
###############################################################################

import matplotlib.pyplot as plt
from utils.maps import RaceMap
from utils.maps import SquareObsMap
from utils.maps import Maze

###############################################################################
# Sampling                                                                    #
###############################################################################

from rrt_methods.rrt_utils import find_bounding_box

###############################################################################
# Plotting                                                                    #
###############################################################################


###############################################################################
# RRT Methods and Variations                                                  #
###############################################################################

# from rrt_methods.rrt import rrt_run
# from rrt_methods.rrt_connect import rrt_run
# from rrt_methods.rrt_star import rrt_run
# from rrt_methods.rrt_star_connect import rrt_run
# from rrt_methods.quick_rrt_star import rrt_run
from rrt_methods.informed_rrt_star import rrt_run

###############################################################################
# Generate Regions                                                            #
###############################################################################

# choice = "rm"
# choice = "sqom"
choice = "maze"

if choice == "rm":
    map = RaceMap()
elif choice == "sqom":
    map = SquareObsMap(10, 100)
elif choice == "maze":
    map = Maze(20)

box_scope = find_bounding_box(map.region)
map.sample_init("box", box_scope)

start = timeit.default_timer()
path = rrt_run(map=map, step_size=20, max_iter=1500, plotting=True)
stop = timeit.default_timer()

print('Time: ', stop - start)


plt.axis("equal")












